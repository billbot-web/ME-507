// ===== DRV8871 Motor (IN1/IN2 + LEDC) + Encoder (PCNT) Live Telemetry =====
// Counts-per-rev math for your setup:
//  - Encoder: 16 pulses/rev on one channel (x1). Using x2 (A rising+falling) → 32 counts/rev at motor shaft.
//  - Gear ratio: 70:1 ⇒ output shaft CPR = 32 * 70 = 2240.
// If you switch to full x4 (count both A & B edges), use 64 * 70 = 4480.

#include <Arduino.h>
#include "Motor.cpp"
#include "Encoder.cpp"

// ---------------- User pins ----------------
#define IN1 13
#define IN2 5
#define ENCODER_A GPIO_NUM_16
#define ENCODER_B GPIO_NUM_17

Motor motor(IN1, IN2, /*LEDC chs*/ 0, 1);
Encoder encoder(ENCODER_A, ENCODER_B, -30000, 30000, /*x2 on A*/ 2);

// ----- Counts-per-rev for your setup -----
static constexpr float COUNTS_PER_REV = 2240.0f;  // x2 on A, 70:1 gear ratio
// If later switched to x4 (A&B both edges), use: 4480.0f

// Telemetry cadence
static const uint32_t SAMPLE_MS = 100;
uint32_t tLast = 0;

void printSample(int32_t dCounts, float dt_s) {
  float cps = dCounts / dt_s;                      // counts per second
  float rpm = (cps / COUNTS_PER_REV) * 60.0f;      // output shaft RPM
  const char* dir = (dCounts > 0) ? "FWD" : (dCounts < 0) ? "REV" : "STOP";
  Serial.printf("ENC dC=%ld  cps=%.1f  rpm=%.2f  dir=%s  total=%ld\n",
                (long)dCounts, cps, rpm, dir, (long)encoder.total());
}

void streamFor(uint32_t ms) {
  uint32_t t0 = millis();
  tLast = millis();
  while (millis() - t0 < ms) {
    if (millis() - tLast >= SAMPLE_MS) {
        float dt = (millis() - tLast) / 1000.0f;
        tLast = millis();
        int32_t dC = encoder.readAndReset();
        printSample(dC, dt);
    }
    delayMicroseconds(10000);
}
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\nMotor + Encoder test (IN1/IN2, PCNT x2, CPR=2240)");

  if (!encoder.begin(PCNT_UNIT_0)) {
    Serial.println("Encoder init FAILED");
    while (1) delay(500);
  }
  Serial.println("Encoder init OK");
}

void loop() {
  Serial.println("Forward 60%");
  motor.setEff(60);
  streamFor(2000);

  Serial.println("Brake");
  motor.stop();
  streamFor(1000);

  Serial.println("Reverse 60%");
  motor.setEff(-60);
  streamFor(2000);

  Serial.println("Brake");
  motor.stop();
  streamFor(1000);
}
