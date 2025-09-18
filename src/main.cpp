// ======================= 프로젝트 목적 =======================
// RC 리시버(무선 조종기에서 나오는 신호)를 읽고,
// PWM(펄스 폭 변조, 신호의 길이로 값이 표현됨) 신호를 받아
// 특정한 "퍼센트 값"과 얼마나 정확히 일치하는지 실험하는 코드.
// 
// LED는 "정해진 값과 딱 맞았을 때만" 무한히 깜빡임.
// 
// 개선점 v5 (구간 매핑):
//  - 이동 평균(32개 샘플을 평균내서 노이즈 제거)
//  - 데드밴드 없음 (중간 값 근처를 뭉뚱그리지 않고 그대로 사용)
//  - 자동 보정 (입력되는 최소/최대 펄스를 자동으로 학습)
//  - -100% ~ +100% 구간을 200개로 잘게 나눠 정확히 구분
//  - 0% 값일 때는 LED를 주황색으로 표시
//  - RC 신호가 끊기면 LED는 꺼짐 (타임아웃 처리)
// ============================================================


// 아두이노와 mbed RTOS 관련 라이브러리 불러오기
#include <Arduino.h>   // 아두이노 기본 함수 (digitalWrite, pinMode 등) 사용
#include "mbed.h"      // 하드웨어 추상화, 시간/스레드 기능 제공
#include "rtos.h"      // RTOS(실시간 운영체제) 관련 기능 (스레드)

// 일부 플랫폼에서는 필요할 수 있는 함수 속성(IRAM_ATTR) 정의
#ifndef IRAM_ATTR
#define IRAM_ATTR
#endif

// 네임스페이스: 코드 짧게 쓰기 위해 사용
using namespace std::chrono;
using namespace std::chrono_literals;
using namespace rtos;


// ============================================================
// -------------------- LED 제어 -------------------------------
// ============================================================

// RGB LED를 켜고 끄는 함수 (rOn/gOn/bOn이 true면 해당 색 켜기)
static inline void setRgb(bool rOn, bool gOn, bool bOn){
  // 아두이노 보드에 따라 LED가 "LOW = 켜짐, HIGH = 꺼짐" 반대로 동작
  digitalWrite(LEDR, rOn ? LOW : HIGH);
  digitalWrite(LEDG, gOn ? LOW : HIGH);
  digitalWrite(LEDB, bOn ? LOW : HIGH);
}

// LED 전부 끄기
static inline void rgbOff(){ setRgb(false, false, false); }

// LED에 원하는 색을 켜거나 끄는 함수
static inline void applyRgb(bool on, bool r, bool g, bool b){
  if (on) setRgb(r, g, b); else rgbOff();
}

// LED 색상 단축 함수 (켜고 끄기 편하도록)
static inline void red(bool on){     applyRgb(on, true,  false, false); }
static inline void yellow(bool on){  applyRgb(on, true,  true,  false); }
static inline void green(bool on){   applyRgb(on, false, true,  false); }
static inline void purple(bool on){  applyRgb(on, true,  false, true ); }
static inline void blue(bool on){    applyRgb(on, false, false, true ); }
static inline void lime(bool on){    applyRgb(on, false, true,  true ); }
// 보드에 따라 실제 "주황" LED가 없으므로, RGB 조합으로 주황 비슷하게 만듦
static inline void orange(bool on){  applyRgb(on, true,  true,  true ); }


// ============================================================
// -------------------- RC 입력 -------------------------------
// ============================================================

// RC 리시버 신호가 들어오는 핀 (PWM 입력)
constexpr pin_size_t RC_PIN = D1;

// RC 신호로 들어올 수 있는 최소/최대 펄스 길이 (마이크로초 단위)
static const uint16_t RC_MIN_US = 800;
static const uint16_t RC_MAX_US = 2200;

// RC 신호가 300ms 동안 들어오지 않으면 "신호 끊김"으로 판단
static const uint32_t RC_TIMEOUT_MS = 300;

// 인터럽트(신호 변화 감지)에서 사용할 전역 변수들
volatile uint32_t gRiseUs = 0;       // PWM 상승(High 시작) 시간 기록
volatile uint16_t gLastPulseUs = 0;  // 마지막으로 측정된 펄스 폭
volatile uint32_t gLastSeenMs = 0;   // 마지막 신호가 들어온 시각

// 인터럽트 핸들러: RC 신호가 High/Low로 바뀔 때마다 실행
void IRAM_ATTR onRcChange(){
  const int lv = digitalRead(RC_PIN);  // 현재 핀 값 읽기
  const uint32_t t = micros();         // 현재 시간을 마이크로초 단위로 읽음

  if (lv == HIGH){
    // 신호가 HIGH로 바뀌면 시작 시각 기록
    gRiseUs = t;
  } else {
    // 신호가 LOW로 바뀌면, 펄스 폭 계산
    uint32_t w = (t - gRiseUs);
    if (w > 0xFFFF) w = 0xFFFF;        // 값이 너무 크면 최대치 제한
    const uint16_t us = (uint16_t)w;

    // 유효한 RC 펄스 범위 안에 들어온 경우만 기록
    if (us >= RC_MIN_US && us <= RC_MAX_US){
      gLastPulseUs = us;              // 측정된 펄스 폭 저장
      gLastSeenMs  = millis();        // 마지막 신호 들어온 시간 저장
    }
  }
}


// ============================================================
// ------------------ 이동 평균 (32샘플) -----------------------
// ============================================================

// 최근 32개 RC 펄스 값을 평균 내서 노이즈 제거
#define AVG_WINDOW 32
volatile uint16_t pulseBuffer[AVG_WINDOW];  // 최근 측정값 저장 버퍼
volatile uint8_t pulseIndex = 0;            // 다음에 기록할 위치

// 새로운 값을 넣고, 평균을 계산하는 함수
uint16_t filterPulse(uint16_t newVal){
  pulseBuffer[pulseIndex++] = newVal;
  if (pulseIndex >= AVG_WINDOW) pulseIndex = 0;

  uint32_t sum = 0; uint8_t count = 0;
  for (uint8_t i=0;i<AVG_WINDOW;i++){
    if (pulseBuffer[i] > 0){ sum += pulseBuffer[i]; count++; }
  }
  if (count == 0) return 0;
  return (uint16_t)(sum / count);
}


// ============================================================
// ------------------ 자동 보정용 변수 -------------------------
// ============================================================

// RC 신호가 들어올 때마다, 최소/최대값 자동 갱신
static uint16_t gMinPulse = 2000; // 지금까지 본 가장 작은 펄스
static uint16_t gMaxPulse = 1000; // 지금까지 본 가장 큰 펄스


// ============================================================
// ------------------ 퍼센트 변환 (구간 매핑) ------------------
// ============================================================

// 펄스 폭을 -100% ~ +100% 값으로 변환하는 함수
int16_t throttlePercentFromUs(uint16_t us){
  if (gMaxPulse <= gMinPulse) return 0; // 아직 보정 안 됨

  int32_t span = gMaxPulse - gMinPulse;   // 전체 범위 길이
  int32_t step = span / 200;              // 200 구간으로 나눔

  if (step <= 0) return 0;

  // 현재 값이 몇 번째 구간인지 계산
  int32_t idx = (us - gMinPulse) / step;

  // -100% ~ +100%로 변환
  int32_t value = idx - 100;

  // 범위 벗어나면 잘라냄
  if (value > 100) value = 100;
  if (value < -100) value = -100;

  return (int16_t)value;
}


// ============================================================
// ------------------ LED 패턴 (정확히 일치만) -----------------
// ============================================================

// 특정 퍼센트 값에 해당하는 LED 색상 정의
struct ValuePattern {
  int16_t value;         // 퍼센트 값
  void (*color)(bool);   // 표시할 색상 함수
};

// 값 → 색상 매핑 테이블
static const ValuePattern VALUE_PATTERNS[] = {
  { 100, red },
  {  99, yellow },
  {  98, green },
  {  97, purple },
  {   0, orange }, // 0%일 때 주황색
  { -50, blue },
  { -99, lime },
};

// 현재 값에 맞는 패턴 찾기
const ValuePattern* findPattern(int16_t value){
  const size_t count = sizeof(VALUE_PATTERNS) / sizeof(VALUE_PATTERNS[0]);
  for (size_t i=0; i<count; ++i){
    if (VALUE_PATTERNS[i].value == value) return &VALUE_PATTERNS[i];
  }
  return nullptr;
}


// ============================================================
// ------------------ Blinker (무한 반복) ----------------------
// ============================================================

// LED 깜빡임 기능 구조체
struct Blinker {
  void (*apply)(bool) = nullptr;  // 어떤 색상 적용할지 함수 포인터
  uint16_t periodMs = 0;          // 깜빡임 주기 (ms)
  uint32_t next = 0;              // 다음 토글할 시간
  bool on = false;                // 현재 켜짐/꺼짐 상태

  // 새 패턴 시작
  void start(void(*func)(bool), uint16_t period){
    stop();
    if (!func || !period) return;
    apply = func;
    periodMs = period;
    on = false;
    next = millis();
  }

  // 정지
  void stop(){
    if (apply) apply(false);
    apply = nullptr;
    on = false;
    periodMs = 0;
  }

  // 주기적으로 실행해 LED 깜빡이게 함
  void run(){
    if (!apply || !periodMs) return;
    const uint32_t now = millis();
    if ((int32_t)(now - next) >= 0){
      do { next += periodMs; } while ((int32_t)(now - next) >= 0);
      on = !on;
      apply(on);
    }
  }
} throttleBlinker;


// ============================================================
// ------------------ RTOS 태스크 ------------------------------
// ============================================================

// 별도 스레드 2개 생성
Thread threadRcInput;  // RC 신호 처리 스레드
Thread threadLed;      // LED 제어 스레드

volatile int16_t gStablePercent = 0x7FFF; // 현재 퍼센트 값 (0x7FFF = 무효)

// RC 입력 처리 스레드
void taskRcInput(){
  while (true){
    noInterrupts();                    // 인터럽트 잠시 차단 (데이터 안전하게 읽기)
    uint16_t us = gLastPulseUs;
    uint32_t seen = gLastSeenMs;
    interrupts();

    if (millis() - seen > RC_TIMEOUT_MS){
      gStablePercent = 0x7FFF; // 신호 끊김
    } else if (us > 0){
      uint16_t avg = filterPulse(us);

      // 자동 보정
      if (avg < gMinPulse) gMinPulse = avg;
      if (avg > gMaxPulse) gMaxPulse = avg;

      int16_t percent = throttlePercentFromUs(avg);
      gStablePercent = percent;
    }

    ThisThread::sleep_for(2ms); // CPU 과부하 방지 (2ms 쉬기)
  }
}

// LED 제어 스레드
void taskLed(){
  int16_t lastPercent = 0x7FFF;
  while (true){
    int16_t now = gStablePercent;
    if (now != lastPercent){
      if (now == 0x7FFF){
        throttleBlinker.stop();
        rgbOff();
      } else {
        const ValuePattern* pattern = findPattern(now);
        if (pattern){
          throttleBlinker.start(pattern->color, 200); // 깜빡임 시작
        } else {
          throttleBlinker.stop();
          rgbOff();
        }
      }
      lastPercent = now;
    }
    throttleBlinker.run();
    ThisThread::sleep_for(20ms); // 20ms 주기 실행
  }
}


// ============================================================
// ------------------ Watchdog -------------------------------
// ============================================================

// 워치독: 프로그램이 멈췄을 때 자동 리셋하는 안전장치
void initWatchdog(uint32_t timeout_ms){
  IWDG1->KR = 0x5555;
  IWDG1->PR = 4;
  IWDG1->RLR = (timeout_ms * 32);
  IWDG1->KR = 0xCCCC;
}

// 워치독 리셋 신호 (주기적으로 호출해야 리셋 안 됨)
void kickWatchdog(){
  IWDG1->KR = 0xAAAA;
}


// ============================================================
// ------------------ 아두이노 엔트리 --------------------------
// ============================================================

// 아두이노 초기화
void setup(){
  // LED 핀 출력 모드로 설정
  pinMode(LEDR, OUTPUT); pinMode(LEDG, OUTPUT); pinMode(LEDB, OUTPUT);
  rgbOff();

  // RC 입력 핀 설정
  pinMode(RC_PIN, INPUT);
  attachInterrupt(RC_PIN, onRcChange, CHANGE); // 신호 변화 감지

  initWatchdog(1000); // 1초 워치독 설정

  // 두 개의 스레드 실행
  threadRcInput.start(taskRcInput);
  threadLed.start(taskLed);
}

// 메인 루프
void loop(){
  static unsigned long lastKick = 0;
  if (millis() - lastKick >= 100){
    kickWatchdog(); // 워치독 리셋
    lastKick = millis();
  }

  yield();                // CPU에 양보
  delayMicroseconds(100); // 아주 짧게 쉬기
}
