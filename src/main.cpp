/**
 * @file main.cpp
 * @brief Main application entry point for ESP32 Motor Control System with IMU Integration
 * 
 * @author Motor Control Development Team
 * @date November 2025
 * @version 2.5
 */

// ===== ESP32 Pan and Tilt Motor Control System with IMU Integration =====
#include <Arduino.h>
#include <cstdint>
#include "DRV883.h"
#include "MotorTask.h"
#include "Encoder.h"
#include "EncoderTask.h"
#include "UITask.h"
#include "IMU_Task.h"
#include <Wire.h>
#include "Adafruit_BNO055.h"
#include "CameraTask.h"
#include "ControllerTask.h"
#include "OV5640_camera.h"
// FreeRTOS APIs used to create tasks/queues
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
// ME507 share utility
#include "taskshare.h"

// FreeRTOS task entry declared in MotorTask.cpp
extern "C" void ui_task_func(void* arg);
extern "C" void imu_task_func(void* arg);
extern "C" void controller_task_func(void* arg);
extern "C" void camera_task_func(void* arg);

// Create separate task functions for tilt and pan to avoid instance_ conflicts
extern "C" void tilt_motor_task_func(void* arg) {
  MotorTask* motorTask = static_cast<MotorTask*>(arg);
  const TickType_t delayTicks = pdMS_TO_TICKS(50);
  for (;;) {
    if (motorTask) motorTask->update();
    vTaskDelay(delayTicks);
  }
}

extern "C" void pan_motor_task_func(void* arg) {
  MotorTask* motorTask = static_cast<MotorTask*>(arg);
  const TickType_t delayTicks = pdMS_TO_TICKS(50);
  for (;;) {
    if (motorTask) motorTask->update();
    vTaskDelay(delayTicks);
  }
}

extern "C" void tilt_encoder_task_func(void* arg) {
  EncoderTask* encoderTask = static_cast<EncoderTask*>(arg);
  const TickType_t delayTicks = pdMS_TO_TICKS(10);
  for (;;) {
    if (encoderTask) encoderTask->update();
    vTaskDelay(delayTicks);
  }
}

extern "C" void pan_encoder_task_func(void* arg) {
  EncoderTask* encoderTask = static_cast<EncoderTask*>(arg);
  const TickType_t delayTicks = pdMS_TO_TICKS(10);
  for (;;) {
    if (encoderTask) encoderTask->update();
    vTaskDelay(delayTicks);
  }
}

// ----------------Pin Declarations ----------------
// ----------------Pan Motor pins ----------------
#define PanIN1 GPIO_NUM_2
#define PanIN2 GPIO_NUM_5
#define nSLP_PIN GPIO_NUM_13
// ---------------- Pan Encoder pins/unit ----------------
#define TiltENCODER_A_GPIO GPIO_NUM_14
#define TiltENCODER_B_GPIO GPIO_NUM_19
#define TiltENCODER_UNIT   PCNT_UNIT_0
// ----------------Tilt Motor pins ----------------
#define TiltIN1 GPIO_NUM_18
#define TiltIN2 GPIO_NUM_23
// ---------------- Tilt Encoder pins/unit ----------------
#define PanENCODER_A_GPIO GPIO_NUM_3
#define PanENCODER_B_GPIO GPIO_NUM_0 //should be GPIO
#define PanENCODER_UNIT   PCNT_UNIT_1
// --------------------------------------------------

// ---------------- Hardware Objects ----------------
//Camera Object 
OV5640Camera camera = OV5640Camera(760);
// Motor: IN1/IN2 on LEDC channels 0 and 1
DRV883 panmotor(PanIN1, PanIN2, /*LEDC chs*/ 0, 1);
Encoder panencoder(PanENCODER_A_GPIO, PanENCODER_B_GPIO, PanENCODER_UNIT);
DRV883 tiltmotor(TiltIN1, TiltIN2, /*LEDC chs*/ 2, 3);
Encoder tiltencoder(TiltENCODER_A_GPIO, TiltENCODER_B_GPIO, TiltENCODER_UNIT);
// IMU instances (global so they can be accessed from loop)
TwoWire I2C = TwoWire(0);
Adafruit_BNO055 bno055_sensor = Adafruit_BNO055(1234, 0x28, &I2C);
// // --------------------------------------------------

// ---------------- Shares and Queues ----------------
//Camera Shares:
Share<int16_t> pan_err{"pan_err"};
Share<int16_t> tilt_err{"tilt_err"};
Share<bool> hasLed{"hasLed"};
Share<uint16_t> ledThreshold{"ledThreshold"};
Share<uint8_t> cam_Mode{"camMode"};
Share<uint8_t> UI_mode{"UI_mode"};
Share<bool> dcalibrate{"dcalibrate"};
// Encoder telemetry shares (declare BEFORE tasks that reference them)
Share<float> tilt_encPosition{"tiltenc_pos"};
Share<float> tilt_encVelocity{"tiltenc_vel"};
Share<float> pan_encPosition{"panenc_pos"};
Share<float> pan_encVelocity{"panenc_vel"};
// Motor reference (0-100)
// Motor reference (-100..100)
Share<int8_t> tilt_vref{"tiltvref"};
Share<int16_t> tilt_posref{"tiltposref"};
Share<int8_t> pan_vref{"panvref"};
Share<int16_t> pan_posref{"panposref"};
// Shared command for encoder/motor (0=STOP, 1=VELOCITY_RUN, 2=POSITION_RUN)
Share<uint8_t> tilt_Cmd{"tiltenc_cmd"};
Share<uint8_t> pan_Cmd{"panenc_cmd"};
// Shared zero flags for encoders
Share<bool> tilt_encZero{"tiltenc_zero"};
Share<bool> pan_encZero{"panenc_zero"};
// D-pad direction shares (-1, 0, 1)
Share<int8_t> dpad_pan{"dpad_pan"};
Share<int8_t> dpad_tilt{"dpad_tilt"};
// Motor test mode control (true = velocity control, false = position control)
Share<bool> motortest_mode{"motortest_mode"};
// IMU mode control (true = send IMU data, false = don't send)
Share<bool> imu_mode{"imu_mode"};
//shares for IMU
// where the 
Share<EulerAngles> g_eulerAngles{"euler_angles"};
Share<GyroData> g_gyroData{"gyro_data"};
Share<AccelData> g_accelData{"accel_data"};
// --------------------------------------------------

// ----------------   Task Objects   ----------------
//Camera task
CameraTask cameraTask(&camera, &pan_err, &tilt_err, &hasLed, &ledThreshold, &cam_Mode);
//Controller task
ControllerTask controllerTask(&pan_err, &tilt_err, &tilt_encPosition, &tilt_encVelocity, &pan_encVelocity, &tilt_Cmd, &pan_Cmd, &cam_Mode, &hasLed, &UI_mode, &dcalibrate, &dpad_pan, &dpad_tilt);
// MotorTask/EncoderTask/UITask instances (pass pointers to the above shares/queue)
MotorTask tiltmotorTask(&tiltmotor, &tilt_Cmd, &tilt_encVelocity, &tilt_vref, &tilt_encPosition, &tilt_posref); 
EncoderTask tiltencoderTask(&tiltencoder, &tilt_encPosition, &tilt_encVelocity, &tilt_Cmd, &tilt_encZero);
MotorTask panmotorTask(&panmotor, &pan_Cmd, &pan_encVelocity, &pan_vref, &pan_encPosition, &pan_posref);
EncoderTask panencoderTask(&panencoder, &pan_encPosition, &pan_encVelocity, &pan_Cmd, &pan_encZero);
// MotorTask/EncoderTask/UITask instances (pass pointers to the above shares/queue)
UITask uiTask(&tilt_encPosition, &tilt_encVelocity, &tilt_vref, &tilt_posref, &tilt_Cmd, &tilt_encZero,
  &pan_encPosition, &pan_encVelocity, &pan_vref, &pan_posref, &pan_Cmd, &pan_encZero,
  &dpad_pan, &dpad_tilt, &imu_mode, &UI_mode, &motortest_mode, &dcalibrate, &g_eulerAngles, &g_gyroData, &g_accelData);
IMUTask imuTask(&bno055_sensor, &g_eulerAngles, &g_gyroData, &g_accelData);
// ---------------------------------------------------

void setup() {
  // Don't initialize Serial (USB) because GPIO1 is used for pan encoder
  Serial.begin(115200);
  delay(300);
  
  // // Use Serial for all debug output instead (RX=GPIO0, TX=GPIO12)
  // Serial.begin(115200, SERIAL_8N1, 0, 12);
  delay(100);
  Serial.println("=== System Starting ===");
  Serial.println("Note: Using Serial for debug (GPIO1 used by pan encoder)");
  
  // Set I2C pins for ESP32
  I2C.setPins(21, 22);
  Serial.println("=== SporTrackr IMU Data Reader ===");
  Serial.println("Initializing IMU...");
  
  // Initialize and start IMU task
  if (imuTask.initIMU()) {
    Serial.println("IMU initialized successfully!");
    delay(300);
    
    // Load saved calibration offsets automatically
    Serial.println("Loading saved calibration offsets...");
    imuTask.loadSavedCalibration();
  } else {
    Serial.println("WARNING: IMU initialization failed!");
    Serial.println("Continuing without IMU...");
  }
  
  // Camera initialization - skip if it fails (GPIO conflicts)
  bool camera_ok = camera.begin();
  if (!camera_ok) {
    Serial.println("WARNING: Camera initialization failed!");
    Serial.println("Continuing without camera...");
  } else {
    Serial.println("Camera initialized successfully");
  }
  
  //enable motors
  pinMode(nSLP_PIN, OUTPUT);
  digitalWrite(nSLP_PIN, HIGH);
  
  // Initialize ESP32 as WiFi Access Point (much easier than connecting to existing networks!)
  // The ESP32 will create its own hotspot that you can connect to from any device
  uiTask.initWebServer("ESP32-MotorControl", "motor123");
  Serial.println("UITask web server initialized as Access Point");

  // Start Motor task (100 ms tick in its own entry function)
  // Start Tilt Motor task
  xTaskCreate(
    tilt_motor_task_func,
    "TiltMotorTask",
    2048,
    &tiltmotorTask,
    2,
    nullptr
  );
  // Start Tilt Encoder task
  xTaskCreate(
    tilt_encoder_task_func,
    "TiltEncoderTask",
    2048,
    &tiltencoderTask,
    5,
    nullptr
  );
  // Start Pan Motor task
  xTaskCreate(
    pan_motor_task_func,
    "PanMotorTask",
    2048,
    &panmotorTask,
    2,
    nullptr
  );
  // Start Pan Encoder task
  xTaskCreate(
    pan_encoder_task_func,
    "PanEncoderTask",
    2048,
    &panencoderTask,
    5,
    nullptr
  );
  // Start UI task
  xTaskCreate(
    ui_task_func,
    "UITask",
    4096,
    &uiTask,
    1,
    nullptr
  );
  //Start IMU task (disabled - I2C errors)
  xTaskCreate(
    imu_task_func,
    "IMUTask",
    4096,
    &imuTask,
    0,
    nullptr
  );
  // Start Controller task
  xTaskCreate(
    controller_task_func,
    "ControllerTask",
    4096,
    &controllerTask,
    3,
    nullptr
  );
  //Start Camera task (temporarily disabled to debug GPIO conflict)
  xTaskCreate(
    camera_task_func,
    "CameraTask",
    4096,
    &cameraTask,
    4,
    nullptr
  );

 }

void loop() {
  // Idle; tasks run under FreeRTOS
  delay(600000000);
}
