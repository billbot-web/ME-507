// ===== DRV883 Motor (IN1/IN2 + LEDC) Live Telemetry =====
#include <Arduino.h>
#include "DRV883.h"
#include "MotorTask.h"
// FreeRTOS APIs used to create tasks
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// FreeRTOS task entry declared in MotorTask.cpp
extern "C" void motor_task_entry(void* arg);

// ---------------- User pins ----------------
#define IN1 22
#define IN2 14

// Motor: IN1/IN2 on LEDC channels 0 and 1
DRV883 motor(IN1, IN2, /*LEDC chs*/ 0, 1);

// MotorTask instance (passes motor pointer; MotorTask does not take ownership)
MotorTask motorTask(&motor);

// Removed encoder instance and telemetry helpers

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\nMotor test (DRV883)");

  // Create a dedicated FreeRTOS task for MotorTask. This runs MotorTask::update()
  // periodically on its own thread so loop() stays responsive.
  xTaskCreate(
    motor_task_entry,     // entry function
    "MotorTask",        // name
    2048,                // stack size (bytes)
    &motorTask,          // parameter (MotorTask instance)
    tskIDLE_PRIORITY + 1,// priority
    nullptr              // task handle
  );
}

void loop() {
  // delay forever; MotorTask is run in its own FreeRTOS task
  delay(600000000);
}
