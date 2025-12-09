/**
 * @file CameraTask.cpp
 * @brief Implementation of LED detection and visual servoing error computation
 * 
 * This file implements real-time computer vision processing for LED target tracking.
 * It captures frames from the OV5640 camera, detects bright LED targets using
 * brightness thresholding, calculates centroid positions, and computes pixel errors
 * for visual servoing control.
 * 
 * Computer Vision Algorithm:
 * 1. Capture frame from OV5640 camera (QVGA or configured resolution)
 * 2. Scan all pixels for brightness above threshold
 * 3. Calculate weighted centroid (center of mass) of bright pixels
 * 4. Compute pixel errors from image center: (centroid - center)
 * 5. Publish errors and detection status via Share<T> objects
 * 6. Optionally stream frame to UI for visualization
 * 
 * Operating Modes:
 * - **WAIT**: Camera idle, no capture (power saving)
 * - **CAP_SEND_ERR**: Capture and compute errors only (fast, no streaming)
 * - **CAP_SEND_RETURN**: Full mode with frame streaming to UI (slower)
 * 
 * Performance Optimization:
 * - CAP_SEND_ERR mode skips frame transmission for maximum frame rate
 * - Brightness threshold configurable via ledThreshold share
 * - Centroid weighted by pixel intensity for sub-pixel accuracy
 * 
 * Data Output:
 * - pan_err: Horizontal pixel error (negative = LED left of center)
 * - tilt_err: Vertical pixel error (negative = LED above center)  
 * - hasLed: Boolean detection flag (true when LED found in frame)
 * 
 * @author Computer Vision Team
 * @date December 2025
 * @version 1.2
 * 
 * @see CameraTask.h for class interface and mode descriptions
 * @see OV5640_camera.h for camera hardware interface
 */

#include <Arduino.h>
#include "../fsm/FSM.h"
#include "../fsm/State.h"
#include "CameraTask.h"

/// @brief Static instance pointer for FSM callback access
CameraTask* CameraTask::instance_ = nullptr;

/**
 * @brief Construct camera task with OV5640 camera and data shares
 * 
 * Initializes the camera task with pointers to the camera hardware interface
 * and shared memory containers for error publication. The camera should already
 * be initialized before constructing this task.
 * 
 * @param camera Pointer to initialized OV5640Camera object
 * @param pan_err Share for horizontal pixel error publication
 * @param tilt_err Share for vertical pixel error publication
 * @param hasLed Share for LED detection status (true when LED visible)
 * @param ledThreshold Share for brightness threshold configuration
 * @param cam_Mode Share for operating mode selection (0=WAIT, 1=ERR, 2=RETURN)
 * @param updateMs Task update period in milliseconds (default 10ms)
 * 
 * @note Camera must be initialized before task creation
 * @note Null camera pointer will be logged as error
 */
CameraTask::CameraTask(OV5640Camera* camera,
                       Share<int16_t>* pan_err,
                       Share<int16_t>* tilt_err,
                       Share<bool>* hasLed,
                       Share<uint16_t>* ledThreshold,
                       Share<uint8_t>* cam_Mode,
                       uint32_t updateMs) noexcept
  : camera_(camera),
    updateMs_(updateMs),
    pan_err_(pan_err),
    tilt_err_(tilt_err),
    hasLed_(hasLed),
    ledThreshold_(ledThreshold),
    cam_Mode_(cam_Mode),
    fsm_(states_, 3)
{
    instance_ = this;
    // Camera should already be initialized before creating this task
    if (camera_) {
        Serial.println("CameraTask: camera pointer received");
    } else {
        Serial.println("CameraTask: ERROR - null camera pointer!");
    }
}

/**
 * @brief FreeRTOS task entry point for camera vision processing
 * 
 * C-style function required by xTaskCreate(). Creates an infinite loop that
 * periodically calls the camera task's update() method to execute the FSM.
 * 
 * Task Loop:
 * 1. Call update() to execute current FSM state
 * 2. Delay for 10ms (100Hz maximum frame rate)
 * 3. Repeat indefinitely
 * 
 * @param arg Pointer to CameraTask object (cast from void*)
 * 
 * @note Fixed 10ms delay provides ~100Hz maximum frame rate
 * @note Actual frame rate depends on camera capture time and processing
 * @note Task never exits - runs for lifetime of system
 */
extern "C" void camera_task_func(void* arg) {
    CameraTask* cameraTask = static_cast<CameraTask*>(arg);
    const TickType_t delayTicks = pdMS_TO_TICKS(10);
    for (;;) {
        if (cameraTask) cameraTask->update();
        vTaskDelay(delayTicks);
    }
}

// ---------------- State implementations ----------------

/**
 * @brief WAIT state - Camera idle, awaiting mode command
 * 
 * In this state, the camera is not capturing frames (power saving mode).
 * The task monitors cam_Mode share for commands to begin capture operations.
 * 
 * State Behavior:
 * 1. Read cam_Mode share to check for activation command
 * 2. If mode = 1: Transition to CAP_SEND_ERR (error computation only)
 * 3. If mode = 2: Transition to CAP_SEND_RETURN (error + frame streaming)
 * 4. If mode = 0: Stay in WAIT (camera idle)
 * 
 * Mode Commands:
 * - cam_Mode = 0: WAIT (camera off)
 * - cam_Mode = 1: CAP_SEND_ERR (fast tracking mode)
 * - cam_Mode = 2: CAP_SEND_RETURN (full mode with UI streaming)
 * 
 * @return Next state ID
 * @retval WAIT Continue waiting (mode = 0)
 * @retval CAP_SEND_ERR Begin fast tracking (mode = 1)
 * @retval CAP_SEND_RETURN Begin full tracking with streaming (mode = 2)
 * 
 * @note No frame capture in this state - camera power conserved
 * @note Error values remain unchanged from last active state
 */
uint8_t CameraTask::exec_wait()
{
    if (!instance_) return WAIT;

    uint8_t mode = 0;
    if (instance_->cam_Mode_) {
        mode = instance_->cam_Mode_->get();
    }

    switch (mode) {
        case 1: 
        Serial.println("CameraTask: Switching to CAP_SEND_ERR");
        return CAP_SEND_ERR; 
        case 2: return CAP_SEND_RETURN;
        default: return WAIT;
    }
}

/**
 * @brief CAP_SEND_ERR state - Fast tracking mode (compute errors only, no streaming)
 * 
 * This is the high-performance tracking mode. It captures frames, detects LED targets,
 * computes pixel errors, and publishes data WITHOUT streaming frames to the UI.
 * This mode provides maximum frame rate for tracking applications.
 * 
 * State Behavior:
 * 1. Check cam_Mode for state transition commands
 * 2. If mode = 0: Return to WAIT
 * 3. If mode = 2: Transition to CAP_SEND_RETURN
 * 4. Capture frame and run LED detection algorithm
 * 5. Compute pan and tilt pixel errors from centroid
 * 6. Publish errors and detection status via shares
 * 7. Apply dynamic threshold from ledThreshold share
 * 8. Return frame buffer to camera driver
 * 9. Stay in CAP_SEND_ERR for continuous operation
 * 
 * Computer Vision Pipeline:
 * - captureAndFindError() performs: frame grab → brightness threshold → centroid → error calc
 * - pan_err negated to match servo direction convention
 * - tilt_err positive when LED below center
 * 
 * Performance:
 * - No frame transmission overhead (faster than CAP_SEND_RETURN)
 * - Suitable for closed-loop visual servoing
 * - Frame rate limited by camera capture time (~10-30ms)
 * 
 * @return Next state ID
 * @retval CAP_SEND_ERR Continue fast tracking (mode = 1)
 * @retval WAIT Stop tracking (mode = 0)
 * @retval CAP_SEND_RETURN Switch to full mode (mode = 2)
 * 
 * @note No frame streaming in this mode - UI shows last streamed frame
 * @note Errors published every cycle for real-time control
 */
uint8_t CameraTask::exec_capture_send_err()
{
    if (!instance_->camera_) return WAIT;
    
    uint8_t mode = 0;
    if (instance_->cam_Mode_) {
        mode = instance_->cam_Mode_->get();
    }

    // Check for mode changes
    switch (mode) {
        case 0:
            Serial.println("CameraTask: Switching to WAIT"); 
            return WAIT;
        case 2: 
            return CAP_SEND_RETURN;
        case 1:
        default:
            break;  // Continue with camera capture
    }

    camera_fb_t* fb = nullptr;
    int16_t pan_err_ = 0;
    int16_t tilt_err_ = 0;
    bool found = false;

    // Capture + compute error
    if (!instance_->camera_->captureAndFindError(fb, pan_err_, tilt_err_, found))
        return CAP_SEND_ERR;

    // Update shares
    if (instance_->pan_err_)  instance_->pan_err_->put(-pan_err_);
    if (instance_->tilt_err_)  instance_->tilt_err_->put(tilt_err_);
    if (instance_->hasLed_) instance_->hasLed_->put(found);
    
    // Push to telemetry queue (non-blocking)
    if (instance_->has_led_queue_) instance_->has_led_queue_->put(found);


    // Apply dynamic threshold
    if (instance_->ledThreshold_) {
        instance_->camera_->setLedThreshold(instance_->ledThreshold_->get());
    }

    // Return frame buffer
    instance_->camera_->returnFrame(fb);

    return CAP_SEND_ERR;
}

/**
 * @brief CAP_SEND_RETURN state - Full mode (errors + frame streaming to UI)
 * 
 * This is the complete visualization mode. It performs all operations of CAP_SEND_ERR
 * plus streams the captured frame with error overlay to the UI via Serial for
 * real-time visual feedback and debugging.
 * 
 * State Behavior:
 * 1. Check cam_Mode for state transition commands
 * 2. If mode = 0: Return to WAIT
 * 3. If mode = 1: Transition to CAP_SEND_ERR (faster mode)
 * 4. Capture frame and run LED detection algorithm
 * 5. Compute and publish pan/tilt pixel errors
 * 6. Apply dynamic threshold configuration
 * 7. Stream frame to Serial with error overlay
 * 8. Return frame buffer to camera driver
 * 9. Stay in CAP_SEND_RETURN for continuous operation
 * 
 * Frame Streaming:
 * - sendFrameWithError() transmits frame with LED centroid marked
 * - Suitable for web UI display and debugging
 * - Adds overhead compared to CAP_SEND_ERR mode
 * 
 * Use Cases:
 * - Initial setup and LED threshold tuning
 * - Debugging tracking problems
 * - Demonstration and visualization
 * - Not recommended for high-speed tracking (use CAP_SEND_ERR instead)
 * 
 * @return Next state ID
 * @retval CAP_SEND_RETURN Continue full mode with streaming (mode = 2)
 * @retval WAIT Stop tracking (mode = 0)
 * @retval CAP_SEND_ERR Switch to fast mode (mode = 1)
 * 
 * @note Frame transmission adds ~50-100ms overhead per frame
 * @note Reduces effective frame rate compared to CAP_SEND_ERR
 * @note UI receives frames for real-time visualization
 */
uint8_t CameraTask::exec_capture_send_return()
{
      if (!instance_->camera_) return WAIT;
      uint8_t mode = 0;
    if (instance_->cam_Mode_) {
        mode = instance_->cam_Mode_->get();
    }

    switch (mode) {
        case 0: return WAIT;
        case 1: return CAP_SEND_ERR;
        default:
            break;  // Continue with camera capture
    }

    camera_fb_t* fb = nullptr;
    int16_t pan_err_ = 0;
    int16_t tilt_err_ = 0;
    bool found = false;

    if (!instance_->camera_->captureAndFindError(fb, pan_err_, tilt_err_, found))
        return CAP_SEND_RETURN;

    if (instance_->pan_err_)  instance_->pan_err_->put(-pan_err_);
    if (instance_->tilt_err_)  instance_->tilt_err_->put(tilt_err_);
    if (instance_->hasLed_) instance_->hasLed_->put(found);

    if (instance_->ledThreshold_) {
        instance_->camera_->setLedThreshold(instance_->ledThreshold_->get());
    }

    instance_->camera_->sendFrameWithError(Serial, fb, found, pan_err_, tilt_err_);
    instance_->camera_->returnFrame(fb);

    return CAP_SEND_RETURN;
}
