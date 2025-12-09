/**
 * @file CameraTask.h
 * @brief Computer vision task for LED detection and tracking error computation
 * 
 * This task implements real-time LED detection and centroid tracking using the OV5640
 * camera module. It captures frames, processes them to find bright LED targets, and
 * computes pixel errors (pan and tilt) from the image center for use in visual servoing.
 * 
 * Key Features:
 * - OV5640 camera frame capture and processing
 * - Brightness-based LED detection with configurable threshold
 * - Centroid calculation for precise target localization
 * - Pixel error computation for visual servoing control
 * - Frame streaming capability for web UI display
 * - FSM-based operation modes for power management
 * 
 * Operating Modes:
 * - **WAIT**: Camera idle, no capture
 * - **CAP_SEND_ERR**: Capture frame, detect LED, compute and publish errors only
 * - **CAP_SEND_RETURN**: Full mode - capture, detect, compute errors, stream frame to UI
 * 
 * Computer Vision Pipeline:
 * 1. Capture frame from OV5640 camera
 * 2. Scan pixels for brightness above threshold
 * 3. Calculate centroid (center of mass) of bright pixels
 * 4. Compute pixel errors: (centroid_x - image_center_x), (centroid_y - image_center_y)
 * 5. Publish errors and detection flag via Share<T> objects
 * 6. Optionally stream frame to web UI
 * 
 * Data Output:
 * - pan_err: Horizontal pixel error (negative = LED left of center)
 * - tilt_err: Vertical pixel error (negative = LED above center)
 * - hasLed: Boolean flag indicating LED detected in current frame
 * 
 * @author Computer Vision Team
 * @date November 2025
 * @version 1.1
 * 
 * @see CameraTask.cpp for LED detection algorithm implementation
 * @see OV5640_camera.h for camera hardware interface
 */

#pragma once
#include <Arduino.h>
#include "../hardware/OV5640_camera.h"
#include "freertos/FreeRTOS.h"
#include "../fsm/State.h"
#include <cstdint>
#include "freertos/task.h"
#include "taskqueue.h"
#include "taskshare.h"  // For Share<T>
#include "../fsm/FSM.h"

/**
 * @class CameraTask
 * @brief Real-time LED detection and visual servoing error computation
 * 
 * Manages OV5640 camera for LED tracking applications. Processes each frame to
 * detect bright LED targets and compute pixel errors for closed-loop visual servoing.
 * Operates in multiple modes for power efficiency and feature selection.
 * 
 * Typical Usage:
 * 1. Construct with OV5640Camera object and data shares
 * 2. Configure LED detection threshold (higher = more selective)
 * 3. Set cam_Mode share to select operating mode
 * 4. Read pan_err/tilt_err shares for visual servoing control
 * 5. Check hasLed share to verify target detection
 * 
 * Performance:
 * - Frame rate depends on camera resolution and mode
 * - CAP_SEND_ERR mode faster (no frame transmission overhead)
 * - CAP_SEND_RETURN mode includes web streaming (slower)
 * 
 * @note LED detection uses simple brightness thresholding (fast but sensitive to ambient light)
 * @note Centroid calculation weighted by pixel brightness for sub-pixel accuracy
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
               Share<int16_t>* pan_err,
               Share<int16_t>* tilt_err,
               Share<bool>*    hasLed,
               Share<uint16_t>* ledThreshold,
               Share<uint8_t>* cam_Mode,
               uint32_t updateMs = 100) noexcept;

    /// Run one FSM cycle
    void update() noexcept { fsm_.run_curstate(); }
    
    // Set telemetry queue for pushing LED detection status to UITask
    void setTelemetryQueue(Queue<bool>* has_led_q) {
      has_led_queue_ = has_led_q;
    }

private:
    OV5640Camera*   camera_;
    uint32_t        updateMs_;
    Share<int16_t>* pan_err_;
    Share<int16_t>* tilt_err_;
    Share<bool>*    hasLed_;
    Share<uint16_t>* ledThreshold_;
    Share<uint8_t>* cam_Mode_;
    
    // Telemetry queue for pushing to UITask (non-blocking)
    Queue<bool>* has_led_queue_ = nullptr;

    // FSM + states
    State state_wait_           {WAIT, "WAIT", &CameraTask::exec_wait};
    State state_cap_send_err_   {CAP_SEND_ERR, "CAP_SEND_ERR", &CameraTask::exec_capture_send_err};
    State state_cap_send_return_{CAP_SEND_RETURN, "CAP_SEND_RETURN", &CameraTask::exec_capture_send_return};
    State* states_[3] = { &state_wait_, &state_cap_send_err_, &state_cap_send_return_ };

    FSM fsm_;

    // --- State function prototypes ---
    static uint8_t exec_wait();
    static uint8_t exec_capture_send_err();
    static uint8_t exec_capture_send_return();

    /// Singleton instance pointer for static callbacks
    static CameraTask* instance_;
};

