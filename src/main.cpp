/**
 * @file main.cpp
 * @brief Main application entry point for ESP32 Motor Control System with IMU Integration
 * 
 * @author Motor Control Development Team
 * @date November 2025
 * @version 2.5
 */

// ===== ESP32 Motor Control System with IMU Integration =====
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
// FreeRTOS APIs used to create tasks/queues
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
// ME507 share utility
#include "taskshare.h"

// FreeRTOS task entry declared in MotorTask.cpp
extern "C" void motor_task_func(void* arg);
extern "C" void encoder_task_func(void* arg);
extern "C" void ui_task_func(void* arg);
extern "C" void imu_task_func(void* arg);

// ---------------- Motor pins ----------------
#define IN1 GPIO_NUM_2
#define IN2 GPIO_NUM_4
// ---------------- Encoder pins/unit (adjust to your wiring) ----------------
#define ENCODER_A_GPIO GPIO_NUM_5
#define ENCODER_B_GPIO GPIO_NUM_18
#define ENCODER_UNIT   PCNT_UNIT_0

// Motor: IN1/IN2 on LEDC channels 0 and 1
DRV883 motor(IN1, IN2, /*LEDC chs*/ 0, 1);
Encoder encoder(ENCODER_A_GPIO, ENCODER_B_GPIO, ENCODER_UNIT);
//IMU:
TwoWire I2C = TwoWire(0);
Adafruit_BNO055 bno055_sensor = Adafruit_BNO055(1234, 0x28, &I2C);

// Encoder telemetry shares (declare BEFORE tasks that reference them)
Share<float> g_encPosition{"enc_pos"};
Share<float>   g_encVelocity{"enc_vel"};
// Motor reference (0-100)
// Motor reference (-100..100)
Share<int8_t> g_vref{"vref"};
Share<int16_t> g_posref{"posref"};

// Shared command for encoder/motor (0=STOP,1=RUN,2=ZERO)
Share<int8_t> g_encCmd{"enc_cmd"};

//Queue for IMU data:
// where the 
Share<EulerAngles> g_eulerAngles{"euler_angles"};
Share<GyroData> g_gyroData{"gyro_data"};
Share<AccelData> g_accelData{"accel_data"};

// MotorTask/EncoderTask/UITask instances (pass pointers to the above shares/queue)
MotorTask motorTask(&motor, &g_encCmd, &g_encVelocity, &g_vref, &g_encPosition, &g_posref); 
EncoderTask encoderTask(&encoder, &g_encPosition, &g_encVelocity, &g_encCmd);
UITask uiTask(&g_encPosition, &g_encVelocity, &g_vref, &g_posref, 
  &g_encCmd, &g_eulerAngles, &g_gyroData, &g_accelData);
IMUTask imuTask(&bno055_sensor, &g_eulerAngles, &g_gyroData, &g_accelData);

void setup() {
  Serial.begin(115200);
  delay(300);
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
  }
  
  // Initialize ESP32 as WiFi Access Point (much easier than connecting to existing networks!)
  // The ESP32 will create its own hotspot that you can connect to from any device
  uiTask.initWebServer("ESP32-MotorControl", "motor123");
  Serial.println("UITask web server initialized as Access Point");

  // Start Motor task (100 ms tick in its own entry function)
  xTaskCreate(
    motor_task_func,
    "MotorTask",
    2048,
    &motorTask,
    2,
    nullptr
  );
  // Start Encoder task (10 ms tick in its own entry function)
  xTaskCreate(
    encoder_task_func,
    "EncoderTask",
    2048,
    &encoderTask,
    3,
    nullptr
  );
  // Start UI task (200 ms tick in its own entry function)
  xTaskCreate(
    ui_task_func,
    "UITask",
    2048,
    &uiTask,
    1,
    nullptr
  );
   // Start UI task (200 ms tick in its own entry function)
  xTaskCreate(
    imu_task_func,
    "IMUTask",
    2048,
    &imuTask,
    0,
    nullptr
  );
 }

void loop() {
  // Idle; tasks run under FreeRTOS
  delay(600000000);
}
