#include <Arduino.h>
#include "esp_camera.h"

// TODO: change these to YOUR wiring
#define CAM_PIN_PWDN   -1      // or GPIO for PWDN if wired
#define CAM_PIN_RESET  -1      // or GPIO for RESET if wired
#define CAM_PIN_XCLK   4     // use if you feed external XCLK; else -1
#define CAM_PIN_SDA   21  // I2C data
#define CAM_PIN_SCL   22

#define CAM_PIN_D9     27 // MSB (sensor D9) 
#define CAM_PIN_D8     26 
#define CAM_PIN_D7     25 
#define CAM_PIN_D6     14 
#define CAM_PIN_D5     35 //input only
#define CAM_PIN_D4     34 //input only
#define CAM_PIN_D3     39 //input only
#define CAM_PIN_D2     36 //LSB (sensor D2) input only
#define CAM_PIN_VSYNC  23
#define CAM_PIN_HREF   18
#define CAM_PIN_PCLK   19 

void setup() {
  Serial.begin(921600);  // try 2000000 later
  delay(1500);

  camera_config_t config = {};
  config.pin_pwdn     = CAM_PIN_PWDN;
  config.pin_reset    = CAM_PIN_RESET;
  config.pin_xclk     = CAM_PIN_XCLK;      // If your camera board has its own crystal and jumper set to INT, you can set this to -1 and omit XCLK
  config.pin_sccb_sda = CAM_PIN_SDA;
  config.pin_sccb_scl = CAM_PIN_SCL;

  config.pin_d0 = CAM_PIN_D2; // LSB (sensor D0)
  config.pin_d1 = CAM_PIN_D3;
  config.pin_d2 = CAM_PIN_D4;
  config.pin_d3 = CAM_PIN_D5;
  config.pin_d4 = CAM_PIN_D6;
  config.pin_d5 = CAM_PIN_D7;
  config.pin_d6 = CAM_PIN_D8;
  config.pin_d7 = CAM_PIN_D9; // MSB (sensor D7)

  config.sccb_i2c_port = 0;

  config.pin_vsync = CAM_PIN_VSYNC;
  config.pin_href  = CAM_PIN_HREF;
  config.pin_pclk  = CAM_PIN_PCLK;

  config.xclk_freq_hz = 12000000;   // safer bring-up
  config.ledc_timer   = LEDC_TIMER_0;
  config.ledc_channel = LEDC_CHANNEL_0;

  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size   = FRAMESIZE_QVGA; //320x240
  config.jpeg_quality = 12; //0-63 lower means higher quality
  config.fb_count     = 1;
  config.fb_location  = CAMERA_FB_IN_DRAM;
  config.grab_mode    = CAMERA_GRAB_LATEST;
  //config.grab_mode    = CAMERA_GRAB_WHEN_EMPTY;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed: 0x%x\n", err);
    while (true) delay(1000);
  }

  Serial.println("Camera OK. Capturing a frame...");

  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Capture failed");
    return;
  }
  Serial.printf("Captured %dx%d JPEG, %u bytes\n", fb->width, fb->height, fb->len);
  esp_camera_fb_return(fb);
}

void loop() {
  delay(50);
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) { Serial.println("Capture failed"); return; }
  // Header "IMG" + 4-byte length (big-endian)
  uint8_t hdr[7] = {'I','M','G',
                    (uint8_t)((fb->len >> 24) & 0xFF),
                    (uint8_t)((fb->len >> 16) & 0xFF),
                    (uint8_t)((fb->len >>  8) & 0xFF),
                    (uint8_t)((fb->len >>  0) & 0xFF)};
  Serial.write(hdr, 7);

  // JPEG payload
  Serial.write(fb->buf, fb->len);
  Serial.flush();

  esp_camera_fb_return(fb);

  // no-op
}
