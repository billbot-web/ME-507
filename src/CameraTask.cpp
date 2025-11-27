/**
 * @file CameraTask.cpp
 * @brief Implementation of CameraTask: a FreeRTOS-based task that uses a simple
 *        WAIT → CAP_SEND_ERR → CAP_SEND_RETURN FSM.
 */

#include <Arduino.h>
#include "FSM.h"
#include "State.h"
#include "CameraTask.h"

// Static instance used by state wrappers
CameraTask* CameraTask::instance_ = nullptr;

/**
 * @brief Construct camera task with shares and queue.
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
 * @brief FreeRTOS task entry; repeatedly calls update() at configured period.
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
 * @brief WAIT state: idle until camMode requests a capture.
 */
uint8_t CameraTask::exec_wait()
{
    if (!instance_) return WAIT;

    uint8_t mode = 0;
    if (instance_->cam_Mode_) {
        mode = instance_->cam_Mode_->get();
    }

    switch (mode) {
        case 1: return CAP_SEND_ERR;
        case 2: return CAP_SEND_RETURN;
        default: return WAIT;
    }
}

/**
 * @brief CAP_SEND_ERR state:
 *        Capture, compute error, update shares, return frame.
 *        DOES NOT send image over serial.
 */
uint8_t CameraTask::exec_capture_send_err()
{
    if (!instance_->camera_) return WAIT;

    camera_fb_t* fb = nullptr;
    int16_t pan_err_ = 0;
    int16_t tilt_err_ = 0;
    bool found = false;

    // Capture + compute error
    if (!instance_->camera_->captureAndFindError(fb, pan_err_, tilt_err_, found))
        return CAP_SEND_ERR;

    // Update shares
    if (instance_->pan_err_)  instance_->pan_err_->put(pan_err_);
    if (instance_->tilt_err_)  instance_->tilt_err_->put(tilt_err_);
    if (instance_->hasLed_) instance_->hasLed_->put(found);

    // Apply dynamic threshold
    if (instance_->ledThreshold_) {
        instance_->camera_->setLedThreshold(instance_->ledThreshold_->get());
    }

    // Return frame buffer
    instance_->camera_->returnFrame(fb);

    return CAP_SEND_ERR;
}

/**
 * @brief CAP_SEND_RETURN state:
 *        Capture + compute error + send frame + return buffer.
 */
uint8_t CameraTask::exec_capture_send_return()
{
    if (!instance_->camera_) return WAIT;

    camera_fb_t* fb = nullptr;
    int16_t pan_err_ = 0;
    int16_t tilt_err_ = 0;
    bool found = false;

    if (!instance_->camera_->captureAndFindError(fb, pan_err_, tilt_err_, found))
        return CAP_SEND_RETURN;

    if (instance_->pan_err_)  instance_->pan_err_->put(pan_err_);
    if (instance_->tilt_err_)  instance_->tilt_err_->put(tilt_err_);
    if (instance_->hasLed_) instance_->hasLed_->put(found);

    if (instance_->ledThreshold_) {
        instance_->camera_->setLedThreshold(instance_->ledThreshold_->get());
    }

    instance_->camera_->sendFrameWithError(Serial, fb, found, pan_err_, tilt_err_);
    instance_->camera_->returnFrame(fb);

    return CAP_SEND_RETURN;
}
