#include "DRV883.h"

/**
 * @brief Constructor configures pins and PWM for DRV883 motor control.
 */
DRV883::DRV883(int in1, int in2, int ledcCh1, int ledcCh2,
               uint32_t pwmFreq, uint8_t resolutionBits)
    : in1_(in1), in2_(in2),
      ch1_(ledcCh1), ch2_(ledcCh2),
      resBits_(resolutionBits),
      maxDuty_((1 << resolutionBits) - 1)
{
    // Configure motor control pins
    pinMode(in1_, OUTPUT);
    pinMode(in2_, OUTPUT);
    digitalWrite(in1_, LOW);
    digitalWrite(in2_, LOW);

    // Configure LEDC PWM
    ledcSetup(ch1_, pwmFreq, resBits_);
    ledcSetup(ch2_, pwmFreq, resBits_);

    ledcAttachPin(in1_, ch1_);
    ledcAttachPin(in2_, ch2_);
}

/**
 * @brief Set motor effort [-100,100]
 */
void DRV883::setEff(int effort) {
    // Clamp effort to valid bounds
    if (effort > 100) effort = 100;
    if (effort < -100) effort = -100;

    uint32_t duty = (abs(effort) * maxDuty_) / 100;

    if (effort > 0) {
        // Forward: PWM on IN1, IN2 = LOW
        ledcWrite(ch1_, duty);
        ledcWrite(ch2_, 0);
    } 
    else if (effort < 0) {
        // Reverse: PWM on IN2, IN1 = LOW
        ledcWrite(ch1_, 0);
        ledcWrite(ch2_, duty);
    } 
    else {
        // Stop: both inputs LOW
        ledcWrite(ch1_, 0);
        ledcWrite(ch2_, 0);
    }
}
/**
 * @brief Stop the motor (zero effort)
 */
void DRV883::stop() {
    setEff(0);
}

/**
 * @brief Brake the motor (active braking by shorting both windings)
 */
void DRV883::brake() {
    // Set both inputs HIGH for active braking (shorts motor windings)
    ledcWrite(ch1_, maxDuty_);
    ledcWrite(ch2_, maxDuty_);
}