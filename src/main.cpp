// main.cpp — RC PWM 수신 (D1) → LED 패턴
// 보드: Portenta H7 (M7) + Portenta HAT Carrier
// 입력: J5-36 (PWM5) = J2-60 = D1   [3.3V PWM]
// 동작: UP(≥1700us) → 빨강 6회, DOWN(≤1300us) → 보라 6회, 그 외 IDLE
// 규약: ISR=기록만, 해석/LED=메인루프, 논블로킹, 업로드 방해 X

#include <Arduino.h>

// --------- (Portenta/mbed) ESP 전용 속성 무시 처리 ----------
#ifndef IRAM_ATTR
#define IRAM_ATTR
#endif

// ---------- LED (Active-LOW: LOW=ON, HIGH=OFF) ----------
static inline void ledInit(){
  pinMode(LEDR, OUTPUT); pinMode(LEDG, OUTPUT); pinMode(LEDB, OUTPUT);
  digitalWrite(LEDR, HIGH); digitalWrite(LEDG, HIGH); digitalWrite(LEDB, HIGH); // all OFF
}
static inline void rgbOff(){ digitalWrite(LEDR,HIGH); digitalWrite(LEDG,HIGH); digitalWrite(LEDB,HIGH); }
static inline void red(bool on){   digitalWrite(LEDR, on?LOW:HIGH); digitalWrite(LEDG,HIGH);         digitalWrite(LEDB,HIGH); }
static inline void magenta(bool on){ // R+B
  digitalWrite(LEDR, on?LOW:HIGH); digitalWrite(LEDG,HIGH);         digitalWrite(LEDB, on?LOW:HIGH);
}

// ---------- 입력 핀 ----------
constexpr pin_size_t RC_PIN = D1;   // ★ 확정: PWM5(J5-36)=D1

// ---------- RC 폭 해석 파라미터 ----------
static const uint16_t RC_MIN_US     = 800;
static const uint16_t RC_MAX_US     = 2200;
static const uint16_t DOWN_MAX_US   = 1300;  // ≤ 이하면 DOWN
static const uint16_t UP_MIN_US     = 1700;  // ≥ 이하면 UP
static const uint32_t RC_TIMEOUT_MS = 300;   // 입력 끊기면 IDLE

// ---------- ISR: 기록만 ----------
volatile uint32_t gRiseUs      = 0; // 마지막 상승 시각
volatile uint16_t gLastPulseUs = 0; // 마지막 완성된 펄스폭(µs)
volatile uint32_t gLastSeenMs  = 0; // 마지막 펄스 완성 시각(ms)

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

// ---------- 논블로킹 점멸기 ----------
struct Blinker {
  uint8_t  remaining = 0;  // 남은 토글 수(ON/OFF 단계)
  uint16_t periodMs  = 0;
  uint32_t next      = 0;
  bool     on        = false;
  void start(uint8_t times, uint16_t period){
    remaining = times * 2; periodMs = period; on = false; next = millis();
  }
  void stop(void(*apply)(bool)){
    remaining = 0; on = false; apply(false);
  }
  void run(void(*apply)(bool)){
    if (!remaining) return;
    const uint32_t now = millis();
    if ((int32_t)(now - next) >= 0){
      do { next += periodMs; } while ((int32_t)(now - next) >= 0);
      on = !on; apply(on);
      if (--remaining == 0){ apply(false); }
    }
  }
} redBlink, magBlink;

// ---------- FSM ----------
enum RcState : uint8_t { ST_IDLE=0, ST_DOWN=1, ST_UP=2 };

static inline RcState classify(uint16_t us){
  if (us <= DOWN_MAX_US) return ST_DOWN;
  if (us >= UP_MIN_US)   return ST_UP;
  return ST_IDLE;
}

void setup(){
  ledInit();

#if defined(INPUT_PULLDOWN)
  pinMode(RC_PIN, INPUT_PULLDOWN);   // 떠있는 경우 LOW로 고정
#else
  pinMode(RC_PIN, INPUT);            // 코어 미지원시 외부 47~100kΩ 풀다운 권장
#endif

  // mbed 코어는 digitalPinToInterrupt 불필요 — 핀 번호로 직접 등록
  attachInterrupt(RC_PIN, onRcChange, CHANGE);
}

void loop(){
  static RcState prev = ST_IDLE;

  // 입력 스냅샷
  noInterrupts();
  uint16_t us   = gLastPulseUs;
  uint32_t seen = gLastSeenMs;
  interrupts();

  // 타임아웃 처리
  if (millis() - seen > RC_TIMEOUT_MS){
    us = 0; // IDLE
  }

  RcState cur = (us==0) ? ST_IDLE : classify(us);

  // 상태 전이 시 패턴 트리거 (명령 누적 금지)
  if (cur != prev){
    switch (cur){
      case ST_UP:
        redBlink.start(6, 200);          // 빨강 6회
        magBlink.stop(magenta);
        break;
      case ST_DOWN:
        magBlink.start(6, 200);          // 보라 6회
        redBlink.stop(red);
        break;
      case ST_IDLE:
      default:
        redBlink.stop(red);
        magBlink.stop(magenta);
        rgbOff();
        break;
    }
    prev = cur;
  }

  // 패턴 실행 (논블로킹)
  redBlink.run(red);
  magBlink.run(magenta);

  // 업로더/USB 우선권 양보
  delay(1);
}
