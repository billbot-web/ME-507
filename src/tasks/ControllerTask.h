/**
 * @file ControllerTask.h
 * @brief High-level FSM controller for camera-based tracking and manual control modes
 * 
 * This task implements the top-level control logic for a pan-tilt camera tracking system.
 * It coordinates between multiple operating modes including automatic LED tracking, manual
 * teleoperation, motor testing, and calibration. The task acts as the "brain" of the system,
 * deciding when to track, when to scan, and how to respond to LED detection loss.
 * 
 * Operating Modes (FSM States):
 * - **CALIBRATE**: Runs encoder calibration sequence (homes motors)
 * - **WAIT**: System idle, motors stopped
 * - **SCAN**: Sweeping pattern to locate LED target
 * - **TRACKR**: Active tracking of detected LED using camera feedback
 * - **TELEOP**: Manual control via D-pad inputs from UI
 * - **MOTOR_TEST**: Open-loop motor testing for diagnostics
 * 
 * Key Features:
 * - Camera error-based tracking with LED detection monitoring
 * - Automatic scan recovery after target loss (5-frame persistence)
 * - Manual control with D-pad for teleoperation
 * - Position safety limits for tilt axis
 * - Mode switching via UI commands
 * 
 * @author Control Systems Team
 * @date November 2025
 * @version 2.1
 * 
 * @see ControllerTask.cpp for FSM state implementations and tracking logic
 * @see MotorTask.h for motor control modes (VRUN, PRUN, CAMERA_PRUN)
 */

#pragma once
#include <Arduino.h>
#include <cstdint>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "../fsm/FSM.h"
#include "../fsm/State.h"
#include "taskshare.h"
#include "taskqueue.h"

class ControllerTask {
public:
    /// FSM state IDs
    enum StateId : uint8_t {
        CALIBRATE = 0,  ///< Calibration state
        WAIT = 1,   ///< Effort = 0, waiting for RUN command
        SCAN  = 2,    ///< Scanning for target
        TRACKR = 3,  ///< Tracking mode
        TELEOP = 4,    ///< Manual teleoperation
        MOTOR_TEST = 5  ///< Motor testing mode
    };

    /**
     * @brief Construct axis controller task.
     * @param errShare     Share for error from camera (pixels, signed)
     * @param effortShare  Share to publish motor effort [-100, 100]
     * @param cmdShare     Command share: 0=STOP, 1=RUN, 2=ZERO
     * @param dpad_pan     D-pad pan direction: -1 (left), 0 (none), 1 (right)
     * @param dpad_tilt    D-pad tilt direction: -1 (down), 0 (none), 1 (up)
     * @param updateMs     Control period in milliseconds
     */
    ControllerTask(Share<int16_t>* pan_err,
                   Share<int16_t>*  tilt_err,
                       Share<float_t>*  tiltpos,
                       Share<int8_t>*  tiltVelo,
                       Share<int8_t>*  panVelo,
                       Share<uint8_t>*  tilt_mode,
                       Share<uint8_t>*  pan_mode,
                       Share<uint8_t>*  Cam_mode,
                       Share<bool>*       hasLed,
                       Share<uint8_t>* UI_mode,
                       Share<bool>* dcalibrate,
                       Share<int8_t>* dpad_pan,
                       Share<int8_t>* dpad_tilt,
                       uint32_t updateMs = 100) noexcept;

    /// Called by FreeRTOS task wrapper
    void update() noexcept { fsm_.run_curstate(); }
    void ConstrainTiltMotor(Share<float_t>* tiltpos) noexcept;
    

    /// Get update period in milliseconds (for task delay)
    uint32_t getUpdateMs() const noexcept { return updateMs_; }

private:
    // Timing
    uint32_t       updateMs_;

    // Shares
    Share<int16_t>* pan_err_;
    Share<int16_t>* tilt_err_;
    Share<float_t>* tiltpos_;
    Share<int8_t>* tiltVelo_;
    Share<int8_t>* panVelo_;
    Share<uint8_t>*  tilt_mode_;
    Share<uint8_t>*  pan_mode_;
    Share<uint8_t>*  Cam_mode_;
    Share<bool>*       hasLed_;
    Share<uint8_t>*  UI_mode_;
    Share<bool>*  dcalibrate_;
    Share<int8_t>* dpad_pan_;
    Share<int8_t>* dpad_tilt_;

    // LED tracking
    uint8_t led_loss_counter_ = 0;
    const uint8_t LED_LOSS_THRESHOLD = 5;  // Consecutive cycles before switching to SCAN

    //Base Scan Velocities
    const int16_t SCAN_PAN_VELO = 0;   // effrtfor pan 50 for tilt
    const int16_t TELEOP_VELO = 17;   // velocity for teleop mode

    // FSM + states
    State state_calibrate_{ CALIBRATE, "CTL_CALIBRATE", &ControllerTask::exec_calibrate };
    State state_wait_{ WAIT, "CTL_WAIT", &ControllerTask::exec_wait };
    State state_scan_{  SCAN,  "CTL_SCAN",  &ControllerTask::exec_scan };
    State state_trackr_{ TRACKR, "CTL_TRACKR", &ControllerTask::exec_trackr };
    State state_teleop_{  TELEOP,  "CTL_TELEOP",  &ControllerTask::exec_teleop };
    State state_motor_test_{ MOTOR_TEST, "CTL_MOTOR_TEST", &ControllerTask::exec_motor_test };
    State* states_[6] = { &state_calibrate_, &state_wait_, &state_scan_, &state_trackr_, &state_teleop_, &state_motor_test_ };
    FSM    fsm_;

    // State functions (static wrappers)
    static uint8_t exec_calibrate();
    static uint8_t exec_wait();
    static uint8_t exec_scan();
    static uint8_t exec_trackr();
    static uint8_t exec_teleop();
    static uint8_t exec_motor_test();

    /// Singleton instance pointer for static callbacks
    static ControllerTask* instance_;


};

