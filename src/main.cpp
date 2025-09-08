#include <Arduino.h>

// Hat Carrier PWM3 → HD J2-65 → D36 (프로젝트 고정 매핑)
// 코어가 D36 심볼을 아직 안 주는 경우가 있음 → 개발용 임시 매핑으로 빌드 통과
#ifndef D36
#  define D36 36
#  warning "D36 not defined by core. Temporarily mapping to 36 (dev only)."
#endif

// Safe 핀 (기본 D39). 코어에 없으면 개발용 임시 매핑
#ifndef D39
#  define D39 39
#  warning "D39 not defined by core. Temporarily mapping to 39 (dev only)."
#endif


// Portenta H7의 온보드 RGB LED 심볼(코어가 제공). 없을 경우 대체 처리.
#ifndef LEDR
#  define LEDR LED_BUILTIN // 최악의 경우 단색 LED로 대체
#  define LEDG LED_BUILTIN
#  define LEDB LED_BUILTIN
#endif

// ============================== 설정 상수 ==============================

// RC 신호(리모컨 CH5) 유효 폭 범위(µs) — 글리치/노이즈 제거용
static const uint16_t RC_MIN_US       = 800;   // 최소 0.8 ms
static const uint16_t RC_MAX_US       = 2200;  // 최대 2.2 ms

// UP/DOWN 판정 경계(µs) — 히스테리시스 포함 운용 권장
static const uint16_t RC_DOWN_MAX_US  = 1300;  // 이하면 DOWN
static const uint16_t RC_UP_MIN_US    = 1700;  // 이하면 IDLE, 이상이면 UP

// RC 타임아웃(ms) — 신호 끊김 시 IDLE 강제 (안전)
static const uint32_t RC_TIMEOUT_MS   = 300;

// LED 점멸 스케줄 기본 (논블로킹, 격자 유지)
static const uint16_t BLINK_HZ        = 3;     // 1초에 3번 on/off → 약 1초간 3회 깜빡
static const uint8_t  BLINK_TIMES     = 3;     // on/off 한 쌍을 3회(총 6토글)

// ============================== 전역 상태 ==============================

// ISR ↔ 메인 간 공유값(기록만). volatile로 표시.
static volatile uint32_t g_rcRiseUs = 0;   // 상승엣지 시각(us)
static volatile uint32_t g_rcWidthUs = 0;  // 측정된 펄스폭(us) — 하강엣지에서 갱신
static volatile uint32_t g_rcEdgeMs  = 0;  // 마지막 유효 폭이 갱신된 시간(ms)

// RC 상태머신
enum RcState { RC_IDLE, RC_UP, RC_DOWN };

// LED 논블로킹 점멸 상태
struct Blink {
  uint8_t  r, g, b;        // 목표 색상
  uint8_t  toggles;        // 남은 토글 횟수(ON/OFF 스텝 수) — times*2
  uint16_t period;         // 토글 주기(ms)
  bool     on;             // 현재 ON/OFF 상태
  bool     active;         // 동작 중 플래그
  uint32_t next;           // 다음 토글 예정 시각(ms)
};
static Blink gBlink = {};  // 제로 초기화

// ============================ 하드웨어 추상화 ============================

// RGB LED 핀 초기화(출력)
static void initRgbPins() {
  pinMode(LEDR, OUTPUT);
  pinMode(LEDG, OUTPUT);
  pinMode(LEDB, OUTPUT);
  digitalWrite(LEDR, LOW);
  digitalWrite(LEDG, LOW);
  digitalWrite(LEDB, LOW);
}

// RGB LED 쓰기(간단 디지털 제어 — 코어별로 HIGH/LOW 극성 같음)
static void writeRgb(uint8_t r, uint8_t g, uint8_t b) {
  digitalWrite(LEDR, r ? HIGH : LOW);
  digitalWrite(LEDG, g ? HIGH : LOW);
  digitalWrite(LEDB, b ? HIGH : LOW);
}

// ========================== Safe 모드 진입 루틴 ==========================

// 부팅 직후 Safe 핀(D39)이 LOW면 — 인터럽트/출력 종료, LED 짧게 점멸만 수행.
// 업로드를 방해하는 블로킹/대기/과도한 delay 금지(논블로킹 점멸만 루프).
static void enterSafeIfHeld() {
  pinMode(D39, INPUT_PULLUP);       // 풀업, GND 쇼트 시 LOW
  if (digitalRead(D39) == LOW) {    // Safe 조건 감지
    detachInterrupt(digitalPinToInterrupt(D36)); // 혹시 대비: 입력 인터럽트 해제
    // CAN 등 외부 버스 모듈 사용 시 종료 — 현재 테스트 코드는 CAN 미사용이지만 규약 유지
    // CAN.end(); // (주석: CAN 사용 시작 전이라 호출 불필요)
    initRgbPins();
    // 빨강 3회 점멸만 반복(논블로킹). 업로드는 여전히 가능.
    gBlink = {};
    gBlink.r = 255; gBlink.g = 0; gBlink.b = 0;
    gBlink.toggles = BLINK_TIMES * 2;
    gBlink.period  = uint16_t(1000 / BLINK_HZ);
    gBlink.on = false;
    gBlink.active = true;
    gBlink.next = millis();

    while (true) {  // Safe 루프 — 논블로킹 점멸만 실행
      uint32_t now = millis();
      if (gBlink.active && (int32_t)(now - gBlink.next) >= 0) {
        do { gBlink.next += gBlink.period; } while ((int32_t)(now - gBlink.next) >= 0);
        gBlink.on = !gBlink.on;
        writeRgb(gBlink.on ? gBlink.r : 0, gBlink.on ? gBlink.g : 0, gBlink.on ? gBlink.b : 0);
        if (--gBlink.toggles == 0) { gBlink.active = false; writeRgb(0,0,0); }
      }
      // USB 업로드/리셋 등 방해하지 않도록 대기/블로킹 호출 없음
    }
  }
}

// ============================ RC 입력 초기화 ============================

// Portenta mbed 코어가 INPUT_PULLDOWN 지원 시 활성 — 미지원이면 외부 100kΩ 풀다운 권장
static void rcInputInit() {
#if defined(INPUT_PULLDOWN)
  pinMode(D36, INPUT_PULLDOWN);   // 입력이 분리될 경우 부동 방지
#else
  pinMode(D36, INPUT);            // (외부 100kΩ → GND 권장)
#endif
  // 엣지 변화 시 인터럽트 — ISR에서는 기록만(상승: 시작시각, 하강: 폭 계산)
  attachInterrupt(digitalPinToInterrupt(D36), [] {
    uint32_t t = micros();                  // 현재 µs
    if (digitalRead(D36)) {                 // 상승엣지: HIGH로 전환됨
      g_rcRiseUs = t;                       // 시작 시각 기록
    } else {                                // 하강엣지: LOW로 전환됨
      uint32_t w = t - g_rcRiseUs;          // 펄스폭 계산(µs)
      if (w > RC_MIN_US && w < RC_MAX_US) { // 글리치/노이즈 제거
        g_rcWidthUs = w;                    // 유효 폭 저장(기록만)
        g_rcEdgeMs  = millis();             // 마지막 갱신 시각(ms) 기록
      }
    }
  }, CHANGE);
}

// ========================== ISR 스냅샷 유틸리티 ==========================

// ISR에서 갱신 중일 수 있으므로 2개 변수(width, time)를 '원자적으로' 묶어서 읽음
struct RcSnap { uint32_t widthUs; uint32_t edgeMs; };
static RcSnap readRcSnapshot() {
  noInterrupts();                       // 임계구역 시작(다 읽을 때까지 ISR 정지)
  RcSnap s{ g_rcWidthUs, g_rcEdgeMs };  // 쌍으로 읽기
  interrupts();                         // 임계구역 종료
  return s;
}

// RC 신호 유효성 판단(타임아웃+범위 체크)
static bool rcValid(const RcSnap& s, uint32_t nowMs) {
  if ((nowMs - s.edgeMs) > RC_TIMEOUT_MS) return false;           // 최신 아님(끊김)
  if (s.widthUs < RC_MIN_US || s.widthUs > RC_MAX_US) return false; // 비정상 폭
  return true;
}

// 폭→상태 해석(히스테리시스 경계 사용)
static RcState decideState(const RcSnap& s) {
  if (s.widthUs >= RC_UP_MIN_US)   return RC_UP;     // 넓은 폭 → 스위치 UP
  if (s.widthUs <= RC_DOWN_MAX_US) return RC_DOWN;   // 좁은 폭 → 스위치 DOWN
  return RC_IDLE;                                    // 중간 → IDLE/중립
}

// ========================== 논블로킹 LED 스케줄러 ==========================

// 새로운 점멸 패턴 설정(이전 패턴 즉시 취소 — 명령 큐 누적 금지)
static void setBlink(uint8_t r, uint8_t g, uint8_t b, uint8_t times, uint16_t hz) {
  gBlink.r = r; gBlink.g = g; gBlink.b = b;
  gBlink.toggles = times * 2;                 // on/off 쌍
  gBlink.period  = uint16_t(1000 / hz);       // 토글 간격(ms)
  gBlink.on = false;
  gBlink.active = true;
  gBlink.next = millis();
}

// 주기 격자(드리프트 0)로 점멸 실행
static void runBlink() {
  if (!gBlink.active) return;
  uint32_t now = millis();
  if ((int32_t)(now - gBlink.next) >= 0) {
    do { gBlink.next += gBlink.period; } while ((int32_t)(now - gBlink.next) >= 0);
    gBlink.on = !gBlink.on;
    writeRgb(gBlink.on ? gBlink.r : 0, gBlink.on ? gBlink.g : 0, gBlink.on ? gBlink.b : 0);
    if (--gBlink.toggles == 0) { gBlink.active = false; writeRgb(0,0,0); }
  }
}

// ================================ setup/loop ================================

void setup() {
  // 0) Safe 모드 판단 — GND 쇼트 시 즉시 Safe 진입(업로드 방해 없음)
  enterSafeIfHeld();

  // 1) LED 핀 초기화
  initRgbPins();

  // 2) RC 입력 초기화(D36 CHANGE 인터럽트). ISR은 '기록만'.
  rcInputInit();

  // 3) (테스트 가시화) 부팅 시 LED 모두 Off
  writeRgb(0,0,0);

#ifdef DEBUG
  Serial.begin(115200);   // 로그 필요 시만 활성화. while(!Serial) 금지(업로드 방해)
#endif
}

void loop() {
  // 1) 시각 캡처
  const uint32_t nowMs = millis();

  // 2) ISR 기록값을 '원자 스냅샷'으로 읽기
  const RcSnap s = readRcSnapshot();

  // 3) 유효 신호인지 검사(타임아웃/범위)
  const bool ok = rcValid(s, nowMs);

  // 4) 상태 해석(UP/DOWN/IDLE)
  static RcState prev = RC_IDLE;    // 직전 상태 기억
  RcState cur = RC_IDLE;
  if (ok) {
    cur = decideState(s);
  } else {
    cur = RC_IDLE;                  // 끊김/비정상 → 안전상태
  }

  // 5) 상태 전이 시 점멸 패턴 갱신(명령 큐 누적 금지: 이전 패턴 즉시 취소 후 새 패턴)
  if (cur != prev) {
    // 전이 시 기존 점멸 중단
    gBlink.active = false; writeRgb(0,0,0);

    // 요구사항: UP시 초록 1초간 3번, DOWN시 빨강 3번, IDLE은 끔
    if (cur == RC_UP)    setBlink(0,255,0, BLINK_TIMES, BLINK_HZ);   // 초록 점멸
    if (cur == RC_DOWN)  setBlink(255,0,0, BLINK_TIMES, BLINK_HZ);   // 빨강 점멸
    if (cur == RC_IDLE)  /* no-op (꺼짐 유지) */ ;

    prev = cur;
  }

  // 6) 논블로킹 점멸 스케줄러 실행(드리프트 0)
  runBlink();

  // 7) (확장 여지) — 여기서 50 ms 격자 스케줄러를 붙여 CAN 등 확장 가능.
  //    예시:
  //    static uint32_t nextCan = nowMs + 50;
  //    if ((int32_t)(nowMs - nextCan) >= 0) {
  //      do { nextCan += 50; } while ((int32_t)(nowMs - nextCan) >= 0);
  //      // ... CAN 송신 주기 작업 ...
  //    }

#ifdef DEBUG
  // 디버그: 최소한의 로그만 (주기적으로 보고 싶으면 격자 사용)
  // static uint32_t nextLog = 0;
  // if ((int32_t)(nowMs - nextLog) >= 0) {
  //   nextLog = nowMs + 500;
  //   Serial.print("RC width(us)="); Serial.print(s.widthUs);
  //   Serial.print(" valid="); Serial.print(ok);
  //   Serial.print(" state="); Serial.println(cur==RC_UP?"UP":(cur==RC_DOWN?"DOWN":"IDLE"));
  // }
#endif
}
