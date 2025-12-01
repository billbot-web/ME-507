/**
 * @file ControllerTask.cpp
 * @brief Implementation of ControllerTask: single-axis PID controller for motor effort.
 */

#include <Arduino.h>
#include "ControllerTask.h"
#include "taskshare.h"
#include "PID.h"

// Static instance used by state wrappers
ControllerTask* ControllerTask::instance_ = nullptr;

/**
 * @brief Construct axis controller task with error/effort shares and command share.
 */
ControllerTask::ControllerTask(Share<int16_t>* pan_err,
                                       Share<int16_t>*  tilt_err,
                                       Share<float_t>*  tiltVelo,
                                       Share<float_t>*  panVelo_,
                                       Share<uint8_t>* tilt_mode,
                                       Share<uint8_t>* pan_mode,
                                       Share<uint8_t>* Cam_mode,
                                       Share<bool>*       hasLed,
                                       Share<uint8_t>*   UI_mode,
                                       Share<bool>*   dcalibrate,
                                       Share<int8_t>*  dpad_pan,
                                       Share<int8_t>*  dpad_tilt,
                                       uint32_t updateMs) noexcept
  : updateMs_(updateMs),
    pan_err_(pan_err),
    tilt_err_(tilt_err),
    tiltVelo_(tiltVelo),
    panVelo_(panVelo_),
    tilt_mode_(tilt_mode),
    pan_mode_(pan_mode),
    Cam_mode_(Cam_mode),
    hasLed_(hasLed),
    UI_mode_(UI_mode),
    dcalibrate_(dcalibrate),
    dpad_pan_(dpad_pan),
    dpad_tilt_(dpad_tilt),

    fsm_(states_, 2)
{
    instance_ = this;
    // Initialize output to zero effort to both motors
    if (panVelo_) panVelo_->put(0);
    if (tiltVelo_) tiltVelo_->put(0);
}

/**
 * @brief FreeRTOS task entry; repeatedly calls update() at configured period.
 */
extern "C" void controller_task_func(void* arg) {
    ControllerTask* ctl = static_cast<ControllerTask*>(arg);
    const TickType_t delayTicks = pdMS_TO_TICKS(ctl->getUpdateMs()); // 100 Hz default
    for (;;) {
        if (ctl) ctl->update();
        vTaskDelay(delayTicks);
    }
}
// ---------------------------------------------------------------------------
// STATE: CTL_WAIT
// Effort forced to 0, wait for RUN/ZERO commands.
// ---------------------------------------------------------------------------
uint8_t ControllerTask::exec_calibrate()
{
    if (!instance_) return CALIBRATE;

    // Ensure effort is zero in WAIT
    if (instance_->panVelo_) instance_->panVelo_->put(0);
    if (instance_->tiltVelo_) instance_->tiltVelo_->put(0);
    
    // //implement calibration routine here later
    // // Check for dcalibrate command if done calibrating go to wait
    // if (instance_->dcalibrate_) {
    //     bool dcalibrate = instance_->dcalibrate_->get();
    //     if (dcalibrate) {
    //         return WAIT;
    //     }
    // }
    // return CALIBRATE;
    // 
    return WAIT;
}
uint8_t ControllerTask::exec_wait()
{
    if (!instance_) return WAIT;

    // Ensure effort is zero in WAIT
    if (instance_->panVelo_) instance_->panVelo_->put(0);
    if (instance_->tiltVelo_) instance_->tiltVelo_->put(0);

    // Read latest command form UI (non-blocking)
    int8_t cmd = 0;
    if (instance_->UI_mode_) {
        cmd = instance_->UI_mode_->get();
    }

    switch (cmd) {
        case 2: // run in telop mode
            return TELEOP;

        case 1: //run in scan mode
            // set scan velocities
            if (instance_->panVelo_) instance_->panVelo_->put(instance_->SCAN_PAN_VELO);
            if (instance_->tiltVelo_) instance_->tiltVelo_->put(instance_->SCAN_PAN_VELO);
            return SCAN;

        case 0:
        default:
            break;
    }

    return WAIT;
}

// ---------------------------------------------------------------------------
// STATE: exec_scan
// Scan for target, track when found.
// ---------------------------------------------------------------------------
uint8_t ControllerTask::exec_scan()
{
    if (!instance_) return WAIT;

    // Handle commands while running
    int8_t cmd = 1;
    if (instance_->UI_mode_) {
        cmd = instance_->UI_mode_->get();
    }

    switch (cmd) {
        case 0: // stop go back to wait
            //set mototo modes
            // set motor efforts to zero
            if (instance_->pan_mode_) instance_->pan_mode_->put(0);
            if (instance_->tilt_mode_) instance_->tilt_mode_->put(0);
            if (instance_->panVelo_) instance_->panVelo_->put(0);
            if (instance_->tiltVelo_) instance_->tiltVelo_->put(0);
            return WAIT;

        case 2: //go to teleop mode
            return TELEOP;
    }
    // Check if LED is found to switch to TRACKR
    if (instance_->hasLed_) {
        bool led_found = instance_->hasLed_->get();
        if (led_found) {
            return TRACKR;
        }
    }
    // Continue scanning if no led was found
    return SCAN;  
}

uint8_t ControllerTask::exec_trackr()
{
    if (!instance_) return WAIT;

    // Handle commands while running
    int8_t cmd = 1;
    if (instance_->UI_mode_) {
        cmd = instance_->UI_mode_->get();
    }

    switch (cmd) {
        case 0: // STOP
            // set motor efforts to zero
            if (instance_->panVelo_) instance_->panVelo_->put(0);
            if (instance_->tiltVelo_) instance_->tiltVelo_->put(0);
            //set motor modes to 0
            if (instance_->pan_mode_) instance_->pan_mode_->put(0);
            if (instance_->tilt_mode_) instance_->tilt_mode_->put(0);
            return WAIT;

        case 2: // ZERO integrator but keep running
            break;

        case 1:
        default:
            break;
    }
    // Check if LED was lost to switch to SCAN
    if (instance_->hasLed_) {
        bool led_found = instance_->hasLed_->get();
        if (!led_found) {
            if (instance_->panVelo_) instance_->panVelo_->put(instance_->SCAN_PAN_VELO);
            if (instance_->tiltVelo_) instance_->tiltVelo_->put(instance_->SCAN_PAN_VELO);
            return SCAN;
        }
    }
    // Continue tracking if LED is still in frame
    return TRACKR;  
}

uint8_t ControllerTask::exec_teleop()
{
    if (!instance_) return WAIT;

    // Handle commands while running
    int8_t cmd = 1;
    if (instance_->UI_mode_) {
        cmd = instance_->UI_mode_->get();
    }
    // Read dpad inputs
    if (instance_->dpad_pan_ && instance_->dpad_tilt_) {
        int8_t pan_dir = instance_->dpad_pan_->get();
        int8_t tilt_dir = instance_->dpad_tilt_->get();
        // Set velocities based on dpad input
        if (instance_->panVelo_) instance_->panVelo_->put(pan_dir * instance_->SCAN_PAN_VELO); 
        // 15 deg/sec per button press
        if (instance_->tiltVelo_) instance_->tiltVelo_->put(tilt_dir * instance_->SCAN_PAN_VELO);
    }

    switch (cmd) {
        case 0: // STOP
            // set motor efforts to zero
            if (instance_->panVelo_) instance_->panVelo_->put(0);
            if (instance_->tiltVelo_) instance_->tiltVelo_->put(0);
            //set motor modes to 0
            if (instance_->pan_mode_) instance_->pan_mode_->put(0);
            if (instance_->tilt_mode_) instance_->tilt_mode_->put(0);
            return WAIT;

        case 1:
            //set scan velocities
            if (instance_->pan_mode_) instance_->pan_mode_->put(1);
            if (instance_->tilt_mode_) instance_->tilt_mode_->put(1);
            if (instance_->panVelo_) instance_->panVelo_->put(instance_->SCAN_PAN_VELO);
            if (instance_->tiltVelo_) instance_->tiltVelo_->put(instance_->SCAN_PAN_VELO);
            return SCAN;
    }

    // --- PID computation ---

    return TELEOP;   // stay in RUN until STOP command
}