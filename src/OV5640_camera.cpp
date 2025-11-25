#include "OV5640_camera.h"

// ---- CAMERA PIN CONFIG (matches your existing wiring) ----
#define CAM_PIN_PWDN   -1
#define CAM_PIN_RESET  -1
#define CAM_PIN_XCLK   -1        // -1 if not using dedicated XCLK

#define CAM_PIN_SDA    21
#define CAM_PIN_SCL    22

#define CAM_PIN_D9     17
#define CAM_PIN_D8     35
#define CAM_PIN_D7     27 
#define CAM_PIN_D6     34
#define CAM_PIN_D5     26
#define CAM_PIN_D4     39
#define CAM_PIN_D3     25
#define CAM_PIN_D2     36

#define CAM_PIN_VSYNC  4
#define CAM_PIN_HREF   15
#define CAM_PIN_PCLK   16

OV5640Camera::OV5640Camera(uint16_t ledThreshold)
: _ledThresh(ledThreshold)
{
    memset(&_config, 0, sizeof(_config));
}

void OV5640Camera::fillDefaultConfig()
{
    _config.pin_pwdn     = CAM_PIN_PWDN;
    _config.pin_reset    = CAM_PIN_RESET;
    _config.pin_xclk     = CAM_PIN_XCLK;
    _config.pin_sccb_sda = CAM_PIN_SDA;
    _config.pin_sccb_scl = CAM_PIN_SCL;

    // Data pins (D0..D7)
    _config.pin_d0 = CAM_PIN_D2;
    _config.pin_d1 = CAM_PIN_D3;
    _config.pin_d2 = CAM_PIN_D4;
    _config.pin_d3 = CAM_PIN_D5;
    _config.pin_d4 = CAM_PIN_D6;
    _config.pin_d5 = CAM_PIN_D7;
    _config.pin_d6 = CAM_PIN_D8;
    _config.pin_d7 = CAM_PIN_D9;

    _config.sccb_i2c_port = 0;

    _config.pin_vsync = CAM_PIN_VSYNC;
    _config.pin_href  = CAM_PIN_HREF;
    _config.pin_pclk  = CAM_PIN_PCLK;

    _config.xclk_freq_hz = 8000000;
    _config.pixel_format = PIXFORMAT_JPEG;
    _config.frame_size   = FRAMESIZE_QQVGA;  // 160x120
    _config.jpeg_quality = 20;
    _config.fb_count     = 1;
    _config.fb_location  = CAMERA_FB_IN_DRAM;
    _config.grab_mode    = CAMERA_GRAB_WHEN_EMPTY;
}

bool OV5640Camera::begin()
{
    fillDefaultConfig();

    esp_err_t err = esp_camera_init(&_config);
    if (err != ESP_OK) {
        Serial.printf("Camera init failed: 0x%x\n", err);
        _initialized = false;
        return false;
    }

    _initialized = true;
    return true;
}

// ───────────────────────────────────────────────────────────────
// Compute LED error from frame center
// ───────────────────────────────────────────────────────────────

bool OV5640Camera::findLedError(const camera_fb_t* fb, int16_t &x_err, int16_t &y_err)
{
    if (!fb) {
        return false;
    }

    int w = fb->width;
    int h = fb->height;

    // Allocate RGB buffer: 3 bytes per pixel
    size_t rgb_size = (size_t)w * (size_t)h * 3;
    uint8_t* rgb_buf = (uint8_t*)malloc(rgb_size);
    if (!rgb_buf) {
        Serial.println("RGB malloc failed");
        return false;
    }

    // Decode JPEG -> RGB888
    if (!fmt2rgb888(fb->buf, fb->len, fb->format, rgb_buf)) {
        Serial.println("fmt2rgb888() failed");
        free(rgb_buf);
        return false;
    }

    uint32_t sum_x = 0;
    uint32_t sum_y = 0;
    uint32_t count = 0;

    uint8_t* p = rgb_buf;

    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            uint8_t r = *p++;
            uint8_t g = *p++;
            uint8_t b = *p++;

            uint16_t bright = (uint16_t)r + (uint16_t)g + (uint16_t)b;

            if (bright > _ledThresh) {
                sum_x += (uint32_t)x;
                sum_y += (uint32_t)y;
                count++;
            }
        }
    }

    free(rgb_buf);

    if (count == 0) {
        // No LED detected
        return false;
    }

    // Centroid in pixel coordinates
    uint16_t cx = (uint16_t)(sum_x / count);
    uint16_t cy = (uint16_t)(sum_y / count);

    // Frame center (integer pixel coords)
    int16_t center_x = (int16_t)(w / 2);
    int16_t center_y = (int16_t)(h / 2);

    // Error = centroid - center (right/down positive)
    x_err = (int16_t)cx - center_x;
    y_err = (int16_t)cy - center_y;

    return true;
}

// ───────────────────────────────────────────────────────────────
// 1) Capture a frame and find error
// ───────────────────────────────────────────────────────────────

bool OV5640Camera::captureAndFindError(camera_fb_t*& fb_out,
                                       int16_t &x_err,
                                       int16_t &y_err,
                                       bool &has_led)
{
    if (!_initialized) {
        Serial.println("captureAndFindError(): camera not initialized");
        return false;
    }

    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) {
        Serial.println("Capture failed");
        return false;
    }

    int16_t xe = 0;
    int16_t ye = 0;
    bool led_found = findLedError(fb, xe, ye);

    fb_out  = fb;
    x_err   = xe;
    y_err   = ye;
    has_led = led_found;

    return true;
}

// ───────────────────────────────────────────────────────────────
// 2) Send a frame (already captured) + error over serial
// ───────────────────────────────────────────────────────────────

bool OV5640Camera::sendFrameWithError(Stream &out,
                                      const camera_fb_t* fb,
                                      bool has_led,
                                      int16_t x_err,
                                      int16_t y_err)
{
    if (!fb) {
        Serial.println("sendFrameWithError(): fb is null");
        return false;
    }

    // Header: "IMG" + 4-byte big-endian length
    uint8_t hdr[7] = {
        'I', 'M', 'G',
        (uint8_t)((fb->len >> 24) & 0xFF),
        (uint8_t)((fb->len >> 16) & 0xFF),
        (uint8_t)((fb->len >>  8) & 0xFF),
        (uint8_t)((fb->len >>  0) & 0xFF)
    };
    out.write(hdr, 7);

    // JPEG payload
    out.write(fb->buf, fb->len);

    // Error coordinates (4 bytes)
    uint8_t coord[4];
    if (!has_led) {
        // Sentinel if LED not found
        coord[0] = 0xFF; coord[1] = 0xFF;
        coord[2] = 0xFF; coord[3] = 0xFF;
    } else {
        coord[0] = (uint8_t)((x_err >> 8) & 0xFF);
        coord[1] = (uint8_t)(x_err & 0xFF);
        coord[2] = (uint8_t)((y_err >> 8) & 0xFF);
        coord[3] = (uint8_t)(y_err & 0xFF);
    }

    out.write(coord, 4);
    out.flush();

    return true;
}

void OV5640Camera::returnFrame(camera_fb_t* fb)
{
    if (fb) {
        esp_camera_fb_return(fb);
    }
}

// ───────────────────────────────────────────────────────────────
// Convenience wrapper: capture + send + return (old behavior)
// ───────────────────────────────────────────────────────────────

bool OV5640Camera::captureLedFrame(Stream &out)
{
    camera_fb_t* fb = nullptr;
    int16_t x_err = 0;
    int16_t y_err = 0;
    bool has_led = false;

    if (!captureAndFindError(fb, x_err, y_err, has_led)) {
        return false;
    }

    bool ok = sendFrameWithError(out, fb, has_led, x_err, y_err);

    returnFrame(fb);
    return ok;
}
