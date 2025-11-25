/**
 * @file AxisControllerTask.cpp
 * @brief Implementation of AxisControllerTask: single-axis PID controller for motor effort.
 */

#include <Arduino.h>
#include "ControllerTask.h"

// Static instance used by state wrappers
AxisControllerTask* AxisControllerTask::instance_ = nullptr;

/**
 * @brief Construct axis controller task with error/effort shares and command share.
 */
AxisControllerTask::AxisControllerTask(Share<int16_t>* errShare,
                                       Share<int8_t>*  effortShare,
                                       Share<int8_t>*  cmdShare,
                                       uint32_t updateMs) noexcept
  : updateMs_(updateMs),
    errShare_(errShare),
    effortShare_(effortShare),
    cmdShare_(cmdShare),
    fsm_(states_, 2)
{
    instance_ = this;

    // Initialize output to zero effort
    if (effortShare_) effortShare_->put(0);
}

/**
 * @brief FreeRTOS task entry; repeatedly calls update() at configured period.
 */
extern "C" void axis_controller_task_func(void* arg) {
    AxisControllerTask* ctl = static_cast<AxisControllerTask*>(arg);
    const TickType_t delayTicks = pdMS_TO_TICKS(10); // 100 Hz default
    for (;;) {
        if (ctl) ctl->update();
        vTaskDelay(delayTicks);
    }
}

// ---------------------------------------------------------------------------
// Helper: clamp motor effort to [-100, 100] and cast to int8_t
// ---------------------------------------------------------------------------
int8_t AxisControllerTask::clamp_effort(float u) noexcept
{
    if (u > 100.0f)  u = 100.0f;
    if (u < -100.0f) u = -100.0f;
    return static_cast<int8_t>(u);
}

// ---------------------------------------------------------------------------
// STATE: CTL_WAIT
// Effort forced to 0, wait for RUN/ZERO commands.
// ---------------------------------------------------------------------------
uint8_t AxisControllerTask::exec_wait() noexcept
{
    if (!instance_) return CTL_WAIT;

    // Ensure effort is zero in WAIT
    if (instance_->effortShare_) instance_->effortShare_->put(0);

    // Read latest command (non-blocking)
    int8_t cmd = 0;
    if (instance_->cmdShare_) {
        cmd = instance_->cmdShare_->get();
    }

    switch (cmd) {
        case 2: // ZERO: reset integrator/derivative, stay in WAIT
            instance_->integral_   = 0.0f;
            instance_->prev_error_ = 0.0f;
            break;

        case 1: // RUN
            return CTL_RUN;

        case 0:
        default:
            break;
    }

    return CTL_WAIT;
}

// ---------------------------------------------------------------------------
// STATE: CTL_RUN
// PID control using error from camera.
// ---------------------------------------------------------------------------
uint8_t AxisControllerTask::exec_run() noexcept
{
    if (!instance_) return CTL_WAIT;

    // Handle commands while running
    int8_t cmd = 1;
    if (instance_->cmdShare_) {
        cmd = instance_->cmdShare_->get();
    }

    switch (cmd) {
        case 0: // STOP
            if (instance_->effortShare_) instance_->effortShare_->put(0);
            return CTL_WAIT;

        case 2: // ZERO integrator but keep running
            instance_->integral_   = 0.0f;
            instance_->prev_error_ = 0.0f;
            break;

        case 1:
        default:
            break;
    }

    // --- PID computation ---

    // Read error (pixels)
    int16_t err_raw = 0;
    if (instance_->errShare_) {
        err_raw = instance_->errShare_->get();
    }

    const float dt = static_cast<float>(instance_->updateMs_) / 1000.0f;
    const float error = static_cast<float>(err_raw);

    // Integrator
    instance_->integral_ += error * dt;
    const float I_MAX = 1000.0f;
    if (instance_->integral_ > I_MAX)  instance_->integral_ = I_MAX;
    if (instance_->integral_ < -I_MAX) instance_->integral_ = -I_MAX;

    // Derivative
    const float derivative = (error - instance_->prev_error_) / dt;
    instance_->prev_error_ = error;

    // PID output
    float u = instance_->kp_ * error
            + instance_->ki_ * instance_->integral_
            + instance_->kd_ * derivative;

    int8_t effort = clamp_effort(u);

    if (instance_->effortShare_) {
        instance_->effortShare_->put(effort);
    }

    return CTL_RUN;   // stay in RUN until STOP command
}
