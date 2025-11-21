/**
 * @file DRV883.h
 * @brief Motor driver class for DRV883 series dual H-bridge motor controllers
 * 
 * This file contains the DRV883 class which provides an interface for controlling
 * DC motors using the Texas Instruments DRV883x series motor driver ICs. The class
 * implements PWM-based speed control with direction control using two input pins.
 * 
 * Key Features:
 * - Bidirectional motor control (-100% to +100% effort)
 * - ESP32 LEDC PWM hardware acceleration 
 * - Configurable PWM frequency and resolution
 * - Active braking support
 * - Hardware protection via DRV883x built-in features
 * 
 * Hardware Requirements:
 * - ESP32 microcontroller
 * - DRV883x motor driver IC (DRV8833, DRV8835, etc.)
 * - Two GPIO pins for motor direction control (IN1, IN2)
 * - External power supply for motor (VM pin on DRV883x)
 * 
 * Wiring:
 * - ESP32 GPIO -> DRV883x IN1 (direction control pin 1)
 * - ESP32 GPIO -> DRV883x IN2 (direction control pin 2) 
 * - Motor + terminal -> DRV883x OUT1
 * - Motor - terminal -> DRV883x OUT2
 * - External V+ -> DRV883x VM (motor power supply)
 * - ESP32 3.3V -> DRV883x VCC (logic power supply)
 * - Common GND between ESP32, DRV883x, and motor supply
 * 
 * @author Motor Control Team
 * @date November 2025
 * @version 1.2
 * 
 * @see https://www.ti.com/lit/ds/symlink/drv8833.pdf DRV8833 Datasheet
 * @see https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/ledc.html ESP32 LEDC Documentation
 */

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
