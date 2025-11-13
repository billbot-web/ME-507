#pragma once
#include <Arduino.h>

/**
 * @class MotorDRV883
 * @brief Simple dual-pin PWM motor driver class for DRV883 series.
 *
 * Controls motor direction using IN1/IN2 pins and speed using ESP32 LEDC PWM.
 * Effort range: -100 to 100 (%)
 * - Positive = forward (PWM on IN1)
 * - Negative = reverse (PWM on IN2)
 * - Zero = motor stopped (both low)
 */
class DRV883 {
public:
    /**
     * @brief Construct a new MotorDRV883 object
     * @param in1 Motor driver IN1 pin
     * @param in2 Motor driver IN2 pin
     * @param ledcCh1 LEDC PWM channel for IN1
     * @param ledcCh2 LEDC PWM channel for IN2
     * @param pwmFreq PWM frequency (Hz)
     * @param resolutionBits PWM resolution (bits)
     */
    DRV883(int in1, int in2, int ledcCh1, int ledcCh2,
                uint32_t pwmFreq = 20000, uint8_t resolutionBits = 10);

    /**
     * @brief Set motor effort
     * @param effort Range [-100,100], percent duty cycle & direction
     */
    void setEff(int effort);

    /// @brief Stop the motor (same as setEff(0))
    void stop();
    /**
     * @brief Brake the motor (active braking by shorting both windings)
     */
    void brake();

private:
    int in1_, in2_;
    int ch1_, ch2_;
    uint8_t resBits_;
    uint32_t maxDuty_;
};
