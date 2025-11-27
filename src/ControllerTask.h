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

class ControllerTask {
public:
    /// FSM state IDs
    enum StateId : uint8_t {
        CALIBRATE = 0,  ///< Calibration state
        WAIT = 1,   ///< Effort = 0, waiting for RUN command
        SCAN  = 2,    ///< Scanning for target (not implemented)
        TRACKR = 3,  ///< Tracking mode
        TELEOP = 4    ///< Manual teleoperation (not implemented)

    };

    /**
     * @brief Construct axis controller task.
     * @param errShare     Share for error from camera (pixels, signed)
     * @param effortShare  Share to publish motor effort [-100, 100]
     * @param cmdShare     Command share: 0=STOP, 1=RUN, 2=ZERO
     * @param updateMs     Control period in milliseconds
     */
    ControllerTask(Share<int16_t>* pan_err,
                   Share<int16_t>*  tilt_err,
                       Share<float_t>*  tiltVelo,
                       Share<float_t>*  panVelo,
                       Share<uint8_t>*  Cam_mode,
                       Share<bool>*       hasLed,
                       Share<uint8_t>* UI_mode,
                       Share<bool>* dcalibrate,
                       uint32_t updateMs = 50) noexcept;

    /// Called by FreeRTOS task wrapper
    void update() noexcept { fsm_.run_curstate(); }

    /// Get update period in milliseconds (for task delay)
    uint32_t getUpdateMs() const noexcept { return updateMs_; }

private:
    // Timing
    uint32_t       updateMs_;

    // Shares
    Share<int16_t>* pan_err_;
    Share<int16_t>* tilt_err_;
    Share<float_t>* tiltVelo_;
    Share<float_t>* panVelo_;
    Share<uint8_t>*  Cam_mode_;
    Share<bool>*       hasLed_;
    Share<uint8_t>*  UI_mode_;
    Share<bool>*  dcalibrate_;

    //Base Scan Velocities
    const int16_t SCAN_PAN_VELO = 15;   // degrees/sec

    // FSM + states
    State state_calibrate_{ CALIBRATE, "CTL_CALIBRATE", &ControllerTask::exec_calibrate };
    State state_wait_{ WAIT, "CTL_WAIT", &ControllerTask::exec_wait };
    State state_scan_{  SCAN,  "CTL_SCAN",  &ControllerTask::exec_scan };
    State state_trackr_{ TRACKR, "CTL_TRACKR", &ControllerTask::exec_trackr };
    State state_teleop_{  TELEOP,  "CTL_TELEOP",  &ControllerTask::exec_teleop };
    State* states_[5] = { &state_calibrate_, &state_wait_, &state_scan_, &state_trackr_, &state_teleop_ };
    FSM    fsm_;

    // State functions (static wrappers)
    static uint8_t exec_calibrate();
    static uint8_t exec_wait();
    static uint8_t exec_scan();
    static uint8_t exec_trackr();
    static uint8_t exec_teleop();

    /// Singleton instance pointer for static callbacks
    static ControllerTask* instance_;


};

