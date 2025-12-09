/**
 * @file EncoderTask.h
 * @brief High-frequency encoder reading task with velocity calculation and telemetry
 * 
 * This task implements continuous encoder position monitoring using the ESP32's PCNT
 * (Pulse Counter) peripheral. It reads quadrature encoder signals at high frequency,
 * calculates angular velocity via numerical differentiation, and publishes both
 * position and velocity data to shared memory for use by control loops.
 * 
 * Key Features:
 * - Hardware-accelerated quadrature decoding via PCNT peripheral
 * - High-rate position updates (default 10ms = 100Hz)
 * - Velocity calculation using first-order finite difference
 * - Zero/home position support for calibration
 * - FSM-based control for clean start/stop behavior
 * - Thread-safe data publication via Share<T> containers
 * 
 * Operating Modes:
 * - **WAIT**: Encoder monitoring paused, awaiting command
 * - **RUN**: Continuous position and velocity measurement
 * 
 * Data Output:
 * - Position: Accumulated angle in degrees from last zero operation
 * - Velocity: Angular rate in degrees/second (calculated from position delta)
 * 
 * Commands:
 * - cmdShare: 0=WAIT (pause), 1=RUN (active)
 * - zeroShare: true=reset position to zero (home/calibrate)
 * 
 * @author Encoder Control Team
 * @date November 2025
 * @version 1.6
 * 
 * @see EncoderTask.cpp for velocity calculation implementation
 * @see Encoder.h for PCNT hardware interface details
 */

#pragma once
#include <Arduino.h>
#include "../hardware/Encoder.h"
#include "freertos/FreeRTOS.h"
#include "../fsm/State.h"
#include <cstdint>
#include "freertos/task.h"
#include "taskqueue.h"
#include "taskshare.h"  // For Share<T>

/**
 * @class EncoderTask
 * @brief High-frequency encoder monitoring with FSM control and velocity estimation
 * 
 * Provides continuous encoder position tracking and velocity calculation for motor
 * control feedback. Uses ESP32 PCNT peripheral for hardware quadrature decoding,
 * enabling high resolution and low CPU overhead.
 * 
 * Typical Usage:
 * 1. Construct with Encoder object and data shares
 * 2. Start FreeRTOS task (runs at updateMs rate)
 * 3. Set cmdShare to 1 to begin monitoring
 * 4. Read position/velocity from shares in control loops
 * 5. Set zeroShare to true to home/calibrate position
 * 
 * @note Default 10ms update rate suitable for most control applications
 * @note Velocity calculated as (position_new - position_old) / dt
 */
class EncoderTask {
public:
    /// State IDs (index into state table)
     /// Singleton instance pointer for static callbacks
    static EncoderTask* instance_;

    enum StateId {
        WAIT = 0,   ///< Waiting for command
        RUN = 1     ///< Reading encoder
    };

  
    /**
     * @brief Construct encoder task with shares and queue.
     * @param encoder Pointer to Encoder object
     * @param positionShare Share for current position
     * @param velocityShare Share for current velocity
     * @param cmdShare Share for commands
     * @param zeroShare Share for zero command (boolean)
     * @param updateMs Update period in milliseconds
     */
    EncoderTask(Encoder* encoder,
                Share<float>* positionShare,
                Share<float>* velocityShare,
                Share<uint8_t>* cmdShare,
                Share<bool>* zeroShare,
                uint32_t updateMs = 10) noexcept;

    void update() noexcept { fsm_.run_curstate(); }
    
    // Set the static instance pointer (called by task function before each update)
    static void set_instance(EncoderTask* inst) { instance_ = inst; }
    
    // Set telemetry queues for pushing data to UITask
    void setTelemetryQueues(Queue<float>* vel_q, Queue<float>* pos_q) {
      vel_queue_ = vel_q;
      pos_queue_ = pos_q;
    }
    
private:
    // Hardware / timing config
    Encoder* encoder_;
    uint32_t updateMs_;

    // Shared resources
    Share<float>* positionShare_;
    Share<float>* velocityShare_;
    Share<uint8_t>* cmdShare_;
    Share<bool>* zeroShare_;
    
    // Telemetry queues for pushing to UITask (non-blocking)
    Queue<float>* vel_queue_ = nullptr;
    Queue<float>* pos_queue_ = nullptr;

    // State management
    static State stateTable_[];

    // FSM + states
    State state_wait_{WAIT, "WAIT", &EncoderTask::exec_wait};
    State state_run_ {RUN,  "RUN",  &EncoderTask::exec_run };
    State* states_[2] = { &state_wait_, &state_run_ };
    FSM fsm_;
    // State functions
    // Static wrappers used for State C-style function pointers
    static uint8_t exec_wait() noexcept;
    static uint8_t exec_run() noexcept;

    // Static instance pointer for state functions
};