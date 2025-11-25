#pragma once

#include <Arduino.h>
#include "esp_camera.h"
#include "img_converters.h"   // fmt2rgb888()

class OV5640Camera {
public:
    // Constructor allows overriding LED threshold if needed
    explicit OV5640Camera(uint16_t ledThreshold = 760);

    // Initialize the camera with hardcoded pin config (matches your wiring)
    bool begin();

    // ─────────────────────────────────────────────
    // 1) Capture + find LED error
    // ─────────────────────────────────────────────
    //
    // - Captures a frame (fb_out)
    // - Computes X_err, Y_err = centroid - frame_center
    // - Sets has_led = true if LED found, false otherwise
    //
    // Caller is responsible for later calling returnFrame(fb_out).
    bool captureAndFindError(camera_fb_t*& fb_out,
                             int16_t &x_err,
                             int16_t &y_err,
                             bool &has_led);

    // ─────────────────────────────────────────────
    // 2) Send frame + error over serial
    // ─────────────────────────────────────────────
    //
    // Sends:
    //   "IMG" + [4-byte JPEG length] + JPEG bytes + [X_err, Y_err]
    //
    // - If has_led == false, sends sentinel 0xFFFF,0xFFFF.
    // - X_err, Y_err are sent as signed 16-bit big-endian.
    bool sendFrameWithError(Stream &out,
                            const camera_fb_t* fb,
                            bool has_led,
                            int16_t x_err,
                            int16_t y_err);

    // Explicitly return a frame buffer to the driver
    void returnFrame(camera_fb_t* fb);

    // Compute X_err, Y_err for the LED relative to frame center
    // (used internally but public in case you want it)
    bool findLedError(const camera_fb_t* fb, int16_t &x_err, int16_t &y_err);

    // Convenience: old one-shot behavior (capture + send + return)
    bool captureLedFrame(Stream &out);

    // Change threshold at runtime
    void setLedThreshold(uint16_t thr) { _ledThresh = thr; }
    uint16_t getLedThreshold() const   { return _ledThresh; }

private:
    camera_config_t _config{};
    bool _initialized = false;
    uint16_t _ledThresh;

    void fillDefaultConfig();
};
