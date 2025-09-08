#include <Arduino.h>
#ifndef IRAM_ATTR
#define IRAM_ATTR
#endif

// ========= 임시 심볼 매핑(개발용) =========
// 코어에 D36/D39 심볼이 없으면 숫자 핀으로 대체해 컴파일만 통과시킵니다.
// 실제 보드에서 반응이 없으면 숫자를 다른 후보(31/33/37 등)로 바꿔 테스트하세요.
#ifndef D36
#warning "D36 not defined by core. Temporarily mapping to 36 (dev only)."
#define D36 36
#endif
#ifndef D39
#warning "D39 not defined by core. Temporarily mapping to 39 (dev only)."
#define D39 39
#endif

// ========= 핀 지정 =========
static constexpr uint8_t PIN_RC   = D36; // R9DS CH5 Signal → J5-31(PWM3) → D36
static constexpr uint8_t PIN_SAFE = 2;   // Safe 점퍼(LOW면 Safe). 지금은 D2로 고정.
static constexpr uint32_t RC_LOSS_MS = 300;
static constexpr uint16_t RC_OFF_US  = 1300; // DOWN
static constexpr uint16_t RC_ON_US   = 1700; // UP

// ========= RGB LED 헬퍼 =========
#if defined(LEDR) && defined(LEDG) && defined(LEDB)
  #define HAVE_RGB 1
#else
  #define HAVE_RGB 0
#endif

static void ledAllOff() {
#if HAVE_RGB
  pinMode(LEDR, OUTPUT); pinMode(LEDG, OUTPUT); pinMode(LEDB, OUTPUT);
  // Portenta RGB: LOW=ON, HIGH=OFF
  digitalWrite(LEDR, HIGH); digitalWrite(LEDG, HIGH); digitalWrite(LEDB, HIGH);
#else
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
#endif
}
static void ledColor(bool r, bool g, bool b) {
#if HAVE_RGB
  digitalWrite(LEDR, r ? LOW : HIGH);
  digitalWrite(LEDG, g ? LOW : HIGH);
  digitalWrite(LEDB, b ? LOW : HIGH);
#else
  digitalWrite(LED_BUILTIN, (r||g||b) ? HIGH : LOW);
#endif
}

// ========= 논블로킹 깜빡이 스케줄러 =========
struct Blink { uint8_t r,g,b; uint8_t toggles; uint32_t period,next; bool on,active; } blink;
static void startBlink(uint8_t r, uint8_t g, uint8_t b, uint8_t blinks_per_sec) {
  blink = {r,g,b, 6, 1000u/blinks_per_sec, millis(), false, true}; // 3번 깜빡=6토글
}
static void runBlink() {
  if (!blink.active) return;
  uint32_t now = millis();
  if ((int32_t)(now - blink.next) >= 0) {
    blink.on = !blink.on;
    ledColor(blink.r && blink.on, blink.g && blink.on, blink.b && blink.on);
    blink.next += blink.period;
    if (blink.toggles) {
      --blink.toggles;
      if (blink.toggles == 0) { blink.active=false; ledAllOff(); }
    }
  }
}

// ========= RC PWM 캡처(인터럽트) =========
volatile uint32_t rcRiseUs = 0;
volatile uint16_t rcWidthUs = 1500;
volatile uint32_t rcEdgeMs  = 0;

static void IRAM_ATTR rcISR() {
  const int lvl = digitalRead(PIN_RC);
  const uint32_t t = micros();
  if (lvl) {
    rcRiseUs = t;
  } else {
    const uint32_t w = t - rcRiseUs;
    if (w >= 500 && w <= 2500) { rcWidthUs = (uint16_t)w; rcEdgeMs = millis(); }
  }
}
static inline bool rcFresh() { return (millis() - rcEdgeMs) < RC_LOSS_MS; }

// ========= 상태 =========
static bool up=false, upPrev=false;

static void updateFromRC() {
  int pos = -1; // 기본: 안전하게 DOWN
  if (rcFresh()) {
    uint16_t w = rcWidthUs;
    if (w > RC_ON_US) pos = +1;           // UP
    else if (w < RC_OFF_US) pos = -1;     // DOWN
    else pos = 0;                         // CENTER → LED off
  }
  up = (pos > 0);

  if (!upPrev && up)  startBlink(0,1,0,3); // UP 에지: 초록 3번
  if ( upPrev && !up) startBlink(1,0,0,3); // DOWN 에지: 빨강 3번
  upPrev = up;

  if (pos <= 0 && !blink.active) ledAllOff();
}

// ========= Safe =========
static bool SAFE=false;
static void enterSafe() {
  SAFE = true;
  detachInterrupt(digitalPinToInterrupt(PIN_RC));
  ledAllOff();
}

// ========= 필수 함수 =========
void setup() {
  pinMode(PIN_SAFE, INPUT_PULLUP);
  if (digitalRead(PIN_SAFE) == LOW) { enterSafe(); return; }

  pinMode(PIN_RC, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_RC), rcISR, CHANGE);
  ledAllOff();
}

void loop() {
  if (SAFE) return;
  updateFromRC();
  runBlink();
  // 업로더 친화용 소량 양보
  delay(1);
}
