// ======================= 프로젝트 목적 =======================
// RC 리시버 PWM 신호(1~2ms 폭)를 읽고,
// 특정 퍼센트 값이 얼마나 정확히 일치하는지 확인하기 위한 연구 전용 코드.
// LED는 지정된 값과 "정확히 일치"할 때만 무한 점멸.
// 개선점 v5 (구간 매핑):
//  - 이동 평균(32샘플)
//  - 데드밴드 없음
//  - 자동 보정 (최소/최대 펄스 갱신, 순서 무관)
//  - 구간 매핑 적용: -100% ~ +100%를 200구간으로 나눠 매칭
//  - 0% 값 → 주황색 표시
//  - RC 타임아웃 처리 (신호 끊기면 LED 꺼짐)
// ============================================================

#include <Arduino.h>
#include "mbed.h"
#include "rtos.h"

#ifndef IRAM_ATTR
#define IRAM_ATTR
#endif

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace rtos;

// ============================================================
// -------------------- LED 제어 -------------------------------
// ============================================================

static inline void setRgb(bool rOn, bool gOn, bool bOn){
  digitalWrite(LEDR, rOn ? LOW : HIGH);
  digitalWrite(LEDG, gOn ? LOW : HIGH);
  digitalWrite(LEDB, bOn ? LOW : HIGH);
}

static inline void rgbOff(){ setRgb(false, false, false); }

static inline void applyRgb(bool on, bool r, bool g, bool b){
  if (on) setRgb(r, g, b); else rgbOff();
}
static inline void red(bool on){     applyRgb(on, true,  false, false); }
static inline void yellow(bool on){  applyRgb(on, true,  true,  false); }
static inline void green(bool on){   applyRgb(on, false, true,  false); }
static inline void purple(bool on){  applyRgb(on, true,  false, true ); }
static inline void blue(bool on){    applyRgb(on, false, false, true ); }
static inline void lime(bool on){    applyRgb(on, false, true,  true ); }
static inline void orange(bool on){  applyRgb(on, true,  true,  true ); } // RGB 조합으로 주황 대체

// ============================================================
// -------------------- RC 입력 -------------------------------
// ============================================================

constexpr pin_size_t RC_PIN = D1;

static const uint16_t RC_MIN_US = 800;
static const uint16_t RC_MAX_US = 2200;
static const uint32_t RC_TIMEOUT_MS = 300;

volatile uint32_t gRiseUs = 0;
volatile uint16_t gLastPulseUs = 0;
volatile uint32_t gLastSeenMs = 0;

void IRAM_ATTR onRcChange(){
  const int lv = digitalRead(RC_PIN);
  const uint32_t t = micros();

  if (lv == HIGH){
    gRiseUs = t;
  } else {
    uint32_t w = (t - gRiseUs);
    if (w > 0xFFFF) w = 0xFFFF;
    const uint16_t us = (uint16_t)w;

    if (us >= RC_MIN_US && us <= RC_MAX_US){
      gLastPulseUs = us;
      gLastSeenMs  = millis();
    }
  }
}

// ============================================================
// ------------------ 이동 평균 (32샘플) -----------------------
// ============================================================

#define AVG_WINDOW 32
volatile uint16_t pulseBuffer[AVG_WINDOW];
volatile uint8_t pulseIndex = 0;

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

static uint16_t gMinPulse = 2000;
static uint16_t gMaxPulse = 1000;

// ============================================================
// ------------------ 퍼센트 변환 (구간 매핑) ------------------
// ============================================================

int16_t throttlePercentFromUs(uint16_t us){
  if (gMaxPulse <= gMinPulse) return 0;

  int32_t span = gMaxPulse - gMinPulse;
  int32_t step = span / 200;

  if (step <= 0) return 0;

  int32_t idx = (us - gMinPulse) / step;
  int32_t value = idx - 100;

  if (value > 100) value = 100;
  if (value < -100) value = -100;

  return (int16_t)value;
}

// ============================================================
// ------------------ LED 패턴 (정확히 일치만) -----------------
// ============================================================

struct ValuePattern {
  int16_t value;
  void (*color)(bool);
};

static const ValuePattern VALUE_PATTERNS[] = {
  { 100, red },
  {  99, yellow },
  {  98, green },
  {  97, purple },
  {   0, orange },
  { -50, blue },
  { -99, lime },
};

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

struct Blinker {
  void (*apply)(bool) = nullptr;
  uint16_t periodMs = 0;
  uint32_t next = 0;
  bool on = false;

  void start(void(*func)(bool), uint16_t period){
    stop();
    if (!func || !period) return;
    apply = func;
    periodMs = period;
    on = false;
    next = millis();
  }

  void stop(){
    if (apply) apply(false);
    apply = nullptr;
    on = false;
    periodMs = 0;
  }

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

Thread threadRcInput;
Thread threadLed;
Thread threadLogger;

volatile int16_t gStablePercent = 0x7FFF;

void taskRcInput(){
  while (true){
    noInterrupts();
    uint16_t us = gLastPulseUs;
    uint32_t seen = gLastSeenMs;
    interrupts();

    if (millis() - seen > RC_TIMEOUT_MS){
      gStablePercent = 0x7FFF;
    } else if (us > 0){
      uint16_t avg = filterPulse(us);
      if (avg < gMinPulse) gMinPulse = avg;
      if (avg > gMaxPulse) gMaxPulse = avg;
      int16_t percent = throttlePercentFromUs(avg);
      gStablePercent = percent;
    }

    ThisThread::sleep_for(2ms);
  }
}

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
          throttleBlinker.start(pattern->color, 200);
        } else {
          throttleBlinker.stop();
          rgbOff();
        }
      }
      lastPercent = now;
    }
    throttleBlinker.run();
    ThisThread::sleep_for(20ms);
  }
}

// ------------------ Logger Task ------------------------------

void taskLogger() {
  uint32_t last3s = millis();

  while (true) {
    uint32_t now = millis();

    if (Serial) {  // USB 연결된 경우에만 출력
      // 3초마다 Min/Max 펄스 출력
      if (now - last3s >= 3000) {
        Serial.print("[");
        Serial.print(now / 1000);
        Serial.print("s] MinPulse=");
        Serial.print(gMinPulse);
        Serial.print(", MaxPulse=");
        Serial.println(gMaxPulse);
        last3s += 3000;
      }
    }

    ThisThread::sleep_for(100ms);
  }
}

// ============================================================
// ------------------ Watchdog -------------------------------
// ============================================================

void initWatchdog(uint32_t timeout_ms){
  IWDG1->KR = 0x5555;
  IWDG1->PR = 4;
  IWDG1->RLR = (timeout_ms * 32);
  IWDG1->KR = 0xCCCC;
}

void kickWatchdog(){
  IWDG1->KR = 0xAAAA;
}

// ============================================================
// ------------------ 아두이노 엔트리 --------------------------
// ============================================================

void setup(){
  Serial.begin(115200);
  pinMode(LEDR, OUTPUT); pinMode(LEDG, OUTPUT); pinMode(LEDB, OUTPUT);
  rgbOff();

  pinMode(RC_PIN, INPUT);
  attachInterrupt(RC_PIN, onRcChange, CHANGE);

  initWatchdog(1000);

  threadRcInput.start(taskRcInput);
  threadLed.start(taskLed);
  threadLogger.start(taskLogger);
}

void loop(){
  static unsigned long lastKick = 0;
  if (millis() - lastKick >= 100){
    kickWatchdog();
    lastKick = millis();
  }

  yield();
  delayMicroseconds(100);
}
