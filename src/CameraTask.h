/**
 * @file CameraTask.h
 * @brief FreeRTOS task wrapper for camera management with FSM-based control
 * 
 * @author Computer Vision Team
 * @date November 2025
 * @version 1.0
 */

#pragma once
#include <Arduino.h>
#include "OV5640_camera.h"
#include "freertos/FreeRTOS.h"
#include "State.h"
#include <cstdint>
#include "freertos/task.h"
#include "taskqueue.h"
#include "taskshare.h"  // For Share<T>
#include "FSM.h"

/**
 * @brief Task wrapper for camera capture and computer vision with simple WAIT/RUN FSM states.
 */
class CameraTask {
public:
    /// State IDs (index into state table)
    enum StateId {
        WAIT = 0,            ///< Waiting for command
        CAP_SEND_ERR = 1,    ///< Capture, compute error only
        CAP_SEND_RETURN = 2  ///< Capture, compute error, send frame, return buffer
    };

    /**
     * @brief Construct camera task with shares and queue.
     * @param camera Pointer to OV5640Camera object
     * @param x_err Share for X error
     * @param y_err Share for Y error
     * @param hasLed Share for LED found flag
     * @param ledThreshold Share for LED detection threshold
     * @param camMode Share for camera mode (0=WAIT, 1=CAP_SEND_ERR, 2=CAP_SEND_RETURN)
     * @param updateMs Update period in milliseconds
     */
    CameraTask(OV5640Camera* camera,
               Share<int16_t>* x_err,
               Share<int16_t>* y_err,
               Share<bool>* hasLed,
               Share<uint8_t>* ledThreshold,
               Share<uint8_t>* camMode,
               uint32_t updateMs = 10) noexcept;

    /// Run one FSM cycle
    void update() noexcept { fsm_.run_curstate(); }

private:
    OV5640Camera*   camera_;
    uint32_t        updateMs_;
    Share<int16_t>* x_err_;
    Share<int16_t>* y_err_;
    Share<bool>*    hasLed_;
    Share<uint8_t>* ledThreshold_;
    Share<uint8_t>* camMode_;

    // FSM + states
    State state_wait_           {WAIT, "WAIT", &CameraTask::exec_wait};
    State state_cap_send_err_   {CAP_SEND_ERR, "CAP_SEND_ERR", &CameraTask::exec_capture_send_err};
    State state_cap_send_return_{CAP_SEND_RETURN, "CAP_SEND_RETURN", &CameraTask::exec_capture_send_return};
    State* states_[3] = { &state_wait_, &state_cap_send_err_, &state_cap_send_return_ };

    FSM fsm_;

    // --- State function prototypes ---
    static uint8_t exec_wait() noexcept;
    static uint8_t exec_capture_send_err() noexcept;
    static uint8_t exec_capture_send_return() noexcept;

    /// Singleton instance pointer for static callbacks
    static CameraTask* instance_;
};

