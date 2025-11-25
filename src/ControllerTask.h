/**
 * @file ControllerTask.h
 * @brief FreeRTOS task wrapper for single-axis PID control using camera error.
 * 
 * This task reads a centroid error (pixels) from a Share<int16_t> and writes a
 * motor effort in the range [-100, 100] to a Share<int8_t>. It uses a simple
 * WAIT/RUN FSM controlled by a command share.
 */

#pragma once
#include <Arduino.h>
#include <cstdint>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "FSM.h"
#include "State.h"
#include "taskshare.h"
#include "taskqueue.h"

class AxisControllerTask {
public:
    /// FSM state IDs
    enum StateId : uint8_t {
        CTL_WAIT = 0,   ///< Effort = 0, waiting for RUN command
        CTL_RUN  = 1    ///< PID control active
    };

    /**
     * @brief Construct axis controller task.
     * @param errShare     Share for error from camera (pixels, signed)
     * @param effortShare  Share to publish motor effort [-100, 100]
     * @param cmdShare     Command share: 0=STOP, 1=RUN, 2=ZERO
     * @param updateMs     Control period in milliseconds
     */
    AxisControllerTask(Share<int16_t>* errShare,
                       Share<int8_t>*  effortShare,
                       Share<int8_t>*  cmdShare,
                       uint32_t updateMs = 10) noexcept;

    /// Called by FreeRTOS task wrapper
    void update() noexcept { fsm_.run_curstate(); }

private:
    // Timing
    uint32_t       updateMs_;

    // Shares
    Share<int16_t>* errShare_;
    Share<int8_t>*  effortShare_;
    Share<int8_t>*  cmdShare_;

    // --- PID state ---
    float kp_  = 0.5f;    ///< Proportional gain
    float ki_  = 0.0f;    ///< Integral gain
    float kd_  = 0.01f;   ///< Derivative gain

    float integral_   = 0.0f;
    float prev_error_ = 0.0f;

    // FSM + states
    State state_wait_{ CTL_WAIT, "CTL_WAIT", &AxisControllerTask::exec_wait };
    State state_run_{  CTL_RUN,  "CTL_RUN",  &AxisControllerTask::exec_run  };
    State* states_[2] = { &state_wait_, &state_run_ };
    FSM    fsm_;

    // State functions (static wrappers)
    static uint8_t exec_wait() noexcept;
    static uint8_t exec_run()  noexcept;

    /// Singleton instance pointer for static callbacks
    static AxisControllerTask* instance_;

    // Helpers
    static int8_t clamp_effort(float u) noexcept;
};

