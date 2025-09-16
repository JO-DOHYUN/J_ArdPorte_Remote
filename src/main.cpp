// main.cpp — RC PWM 수신 (D1) → LED 패턴
// 보드: Portenta H7 (M7) + Portenta HAT Carrier
// 입력: J5-36 (PWM5) = J2-60 = D1   [3.3V PWM]
// 동작: UP(≥1700us) → 빨강 6회, DOWN(≤1300us) → 보라 6회, 그 외 IDLE
// 규약: ISR(인터럽트)=기록만, 해석/LED=메인루프, 논블로킹(블로킹 대기/지연 없음), 업로드 방해 X

#include <Arduino.h>  // Arduino 프레임워크의 기본 함수/타입( pinMode, digitalRead 등 )을 사용하기 위한 헤더

// --------- (Portenta/mbed) ESP 전용 속성 무시 처리 ----------
// IRAM_ATTR: ESP 계열(예: ESP32)에서 ISR을 IRAM(명령 RAM)에 배치하라는 지시자.
// Portenta H7(mbed) 환경에서는 이 매크로가 정의되어 있지 않아 컴파일 에러가 날 수 있으므로,
// 없으면 '빈 매크로'로 정의해서 무시한다. (코드 동작에는 영향 없음)
#ifndef IRAM_ATTR
#define IRAM_ATTR
#endif

// ---------- LED (Active-LOW: LOW=ON, HIGH=OFF) ----------
// Portenta H7의 온보드 RGB LED는 액티브-로우(LOW가 켜짐, HIGH가 꺼짐) 특성을 가진다.
// 아래 유틸 함수들은 이 특성을 반영해서 LED를 초기화/제어한다.
static inline void ledInit(){
  // 세 색(R/G/B) 모두 출력 모드로 설정
  pinMode(LEDR, OUTPUT); pinMode(LEDG, OUTPUT); pinMode(LEDB, OUTPUT);
  // all OFF: 액티브-로우이므로 HIGH를 써서 모두 꺼둔다
  digitalWrite(LEDR, HIGH); digitalWrite(LEDG, HIGH); digitalWrite(LEDB, HIGH);
}
// 세 색을 모두 끄는 헬퍼
static inline void rgbOff(){ digitalWrite(LEDR,HIGH); digitalWrite(LEDG,HIGH); digitalWrite(LEDB,HIGH); }
// '빨강'만 켜거나 끄는 헬퍼. on=true면 LOW(켜짐), on=false면 HIGH(꺼짐)
static inline void red(bool on){   digitalWrite(LEDR, on?LOW:HIGH); digitalWrite(LEDG,HIGH);         digitalWrite(LEDB,HIGH); }
// '보라(자홍)=빨강+파랑'만 켜거나 끄는 헬퍼. 녹색은 항상 꺼짐
static inline void magenta(bool on){
  digitalWrite(LEDR, on?LOW:HIGH); digitalWrite(LEDG,HIGH);         digitalWrite(LEDB, on?LOW:HIGH);
}

// ---------- 입력 핀 ----------
// RC 수신 PWM 신호가 들어오는 실제 핀. 사용자 환경에서 J5-36(PWM5)=D1로 확정되었으므로 D1 사용.
// pin_size_t: Arduino에서 핀 번호를 나타내는 정수 타입(alias). 캐스팅은 명시적 타입 일치용.
constexpr pin_size_t RC_PIN = D1;   // ★ 확정: PWM5(J5-36)=D1

// ---------- RC 폭 해석 파라미터 ----------
// 일반 RC PWM의 펄스폭 유효 범위를 설정(노이즈/잘못된 판별 방지)
static const uint16_t RC_MIN_US     = 800;   // 최소 유효 폭(마이크로초)
static const uint16_t RC_MAX_US     = 2200;  // 최대 유효 폭(마이크로초)
// 채널 상태를 세 구간으로 분리: DOWN / 중립 / UP
static const uint16_t DOWN_MAX_US   = 1300;  // 이 값 이하이면 'DOWN'
static const uint16_t UP_MIN_US     = 1700;  // 이 값 이상이면 'UP'
static const uint32_t RC_TIMEOUT_MS = 300;   // 마지막 유효 펄스 이후 이 시간(ms) 지나면 '입력 끊김'으로 보고 IDLE 처리

// ---------- ISR: 기록만 ----------
// 인터럽트 서비스 루틴에서만 접근되는 최신 측정 결과를 보관하는 공유 변수들.
// 'volatile'로 최적화 방지(컴파일러가 값이 바뀌지 않는다고 가정하지 않도록), ISR/메인루프 간 안전 공유.
// 주의: ISR에서는 '기록만' 수행(무거운 연산/지연/출력 금지: 업로드/시스템 안정성 보장).
volatile uint32_t gRiseUs      = 0; // 마지막 상승 엣지 시각(micros 단위)
volatile uint16_t gLastPulseUs = 0; // 마지막으로 완성된 유효 펄스폭(µs). 0이면 아직 없음/타임아웃으로 간주.
volatile uint32_t gLastSeenMs  = 0; // 마지막 유효 펄스가 완성된 시각(millis 단위)

// onRcChange: RC_PIN의 레벨 변화를 엣지(상승/하강) 단위로 감지.
// 상승(HIGH)이면 시작 시각만 기록하고, 하강(LOW)이면 폭을 계산하여 유효 범위일 때만 결과를 반영한다.
// 이렇게 하면 ISR의 수행 시간이 매우 짧고 예측 가능해져 시스템/업로더를 방해하지 않는다.
void IRAM_ATTR onRcChange(){
  const int lv = digitalRead(RC_PIN);  // 현재 핀 레벨(HIGH/LOW). Arduino 레이어의 빠른 디지털 읽기.
  const uint32_t t = micros();         // 현재 시각(µs). 32비트 wrap-around 허용: 뺄셈으로 안전하게 처리.
  if (lv == HIGH){
    // 상승 엣지: 펄스 시작 시각만 기록
    gRiseUs = t;
  } else {
    // 하강 엣지: 폭 계산 (현재 시각 - 마지막 상승 시각)
    uint32_t w = (t - gRiseUs);        // 언더플로는 32비트 모듈러로 자동 보정됨
    if (w > 0xFFFF) w = 0xFFFF;        // 16비트 범위로 캐스팅 전에 상한 클램프
    const uint16_t us = (uint16_t)w;   // 마이크로초 폭(16비트)
    // 유효 범위에만 반영(노이즈/이상치 무시)
    if (us >= RC_MIN_US && us <= RC_MAX_US){
      gLastPulseUs = us;               // 최신 유효 펄스폭 업데이트
      gLastSeenMs  = millis();         // 유효 펄스가 '완료된' 시각 기록(타임아웃 판정용)
    }
  }
}

// ---------- 논블로킹 점멸기 ----------
// LED 점멸을 '토글 스케줄'로 수행하는 작은 상태기계.
// - remaining: 앞으로 남은 토글 수(ON/OFF 한 번씩 진행되므로 '점멸 횟수×2')
// - periodMs : 토글 간격(ms)
// - next     : 다음 토글 시각(ms)
// - on       : 현재 LED가 켜졌는지(액티브-로우 기준 true=켜짐)
// run()은 '지나갔으면 토글한다' 패턴으로 동작(밀리스 격자, 드리프트 0 유지).
struct Blinker {
  uint8_t  remaining = 0;  // 남은 토글 수(ON/OFF 단계 합계)
  uint16_t periodMs  = 0;  // 토글 간격(ms)
  uint32_t next      = 0;  // 다음 토글 예정 시각(ms)
  bool     on        = false; // 현재 ON 상태(액티브-로우 기준)
  // start: n회 점멸 시작(내부적으로 토글은 2×n회)
  void start(uint8_t times, uint16_t period){
    remaining = times * 2; periodMs = period; on = false; next = millis();
  }
  // stop: 즉시 정지하고 적용 콜백으로 OFF 상태 반영(LED를 꺼둠)
  void stop(void(*apply)(bool)){
    remaining = 0; on = false; apply(false);
  }
  // run: 시간이 되었을 때만 토글. apply는 'LED 제어 함수 포인터'(red/magenta 등)
  void run(void(*apply)(bool)){
    if (!remaining) return;               // 남은 게 없으면 아무것도 안 함(논블로킹)
    const uint32_t now = millis();
    if ((int32_t)(now - next) >= 0){      // now >= next (랩어라운드 안전 비교)
      // 빠르게 지나친 기간도 보정(격자 유지: next += period 반복)
      do { next += periodMs; } while ((int32_t)(now - next) >= 0);
      on = !on;                           // 상태 토글
      apply(on);                          // 실제 LED에 반영
      if (--remaining == 0){              // 토글을 모두 소진했으면
        apply(false);                     // OFF로 마무리
      }
    }
  }
} redBlink, magBlink; // 빨강/보라 점멸기를 별도로 운용(서로 덮어쓰며 큐를 누적하지 않음)

// ---------- FSM ----------
// 입력 폭(us)을 DOWN/IDLE/UP 세 가지 상태로 해석하는 간단한 상태기계.
enum RcState : uint8_t { ST_IDLE=0, ST_DOWN=1, ST_UP=2 };

// 분류 함수: 폭 기준으로 상태 반환
static inline RcState classify(uint16_t us){
  if (us <= DOWN_MAX_US) return ST_DOWN;
  if (us >= UP_MIN_US)   return ST_UP;
  return ST_IDLE; // 중간 범위는 중립
}

void setup(){
  ledInit(); // LED 핀 초기화 및 OFF

#if defined(INPUT_PULLDOWN)
  // 입력 핀 내부 풀다운 사용: 떠있는(floating) 상태에서 노이즈로 오인식하는 것을 방지.
  pinMode(RC_PIN, INPUT_PULLDOWN);
#else
  // 코어가 INPUT_PULLDOWN을 지원하지 않는 드문 경우: 외부 47~100kΩ 풀다운 권장.
  pinMode(RC_PIN, INPUT);
#endif

  // 인터럽트 등록.
  // mbed 기반 Portenta 코어에서는 digitalPinToInterrupt 변환이 필요 없이 '핀 번호'로 직접 등록 가능.
  // 트리거 모드는 CHANGE: 상승/하강 모두 ISR이 호출되어 폭을 측정할 수 있게 한다.
  attachInterrupt(RC_PIN, onRcChange, CHANGE);
}

void loop(){
  static RcState prev = ST_IDLE; // 직전 상태 저장(상태 전이 감지용)

  // ---- (1) 입력 스냅샷 ----
  // ISR에서 갱신되는 공유 변수들을 '한 순간' 값으로 가져오기 위해
  // 짧게 인터럽트를 막고(noInterrupts), 복사 후 즉시 복귀(interrupts).
  // → 일관된 데이터(레이스 조건 방지)를 확보.
  noInterrupts();
  uint16_t us   = gLastPulseUs;  // 마지막 유효 펄스폭(us). 0이면 '없음'으로 취급
  uint32_t seen = gLastSeenMs;   // 마지막 유효 펄스가 발생한 시각(ms)
  interrupts();

  // ---- (2) 타임아웃 처리 ----
  // 일정 시간(RC_TIMEOUT_MS) 동안 유효 펄스가 없으면 입력 끊김으로 보고 IDLE 처리.
  if (millis() - seen > RC_TIMEOUT_MS){
    us = 0; // 0은 아래 분류에서 ST_IDLE로 해석되도록 사용
  }

  // ---- (3) 분류/상태 전이 ----
  RcState cur = (us==0) ? ST_IDLE : classify(us); // 0이면 강제 IDLE

  // 상태가 바뀐 경우에만 LED 패턴을 트리거(명령 큐 누적 방지: 새 패턴이 기존 패턴을 덮어씀).
  if (cur != prev){
    switch (cur){
      case ST_UP:
        // UP: 빨강 6회 점멸. 보라 패턴은 즉시 정지/소거.
        redBlink.start(6, 200);          // 200ms 토글 간격 → 6회 점멸
        magBlink.stop(magenta);          // 보라 패턴 중이었으면 중단하고 OFF
        break;
      case ST_DOWN:
        // DOWN: 보라 6회 점멸. 빨강 패턴은 즉시 정지/소거.
        magBlink.start(6, 200);
        redBlink.stop(red);
        break;
      case ST_IDLE:
      default:
        // IDLE: 모든 패턴 정지, LED 모두 OFF
        redBlink.stop(red);
        magBlink.stop(magenta);
        rgbOff();
        break;
    }
    prev = cur; // 현재 상태를 다음 루프의 '직전 상태'로 저장
  }

  // ---- (4) 패턴 실행 ----
  // 각 점멸기는 자신의 스케줄에 도달하면 토글만 수행(논블로킹).
  redBlink.run(red);
  magBlink.run(magenta);

  // ---- (5) CPU/USB 양보 ----
  // 아주 짧은 지연으로 스케줄러/업로더/USB 스택에 CPU를 양보.
  // (긴 delay나 while 대기는 절대 사용하지 않음: 업로드/디버그 방해 금지 규칙)
  delay(1);
}
