// ===== DRV883 Motor + Encoder + UI Tasks =====
#include <Arduino.h>
#include "DRV883.h"
#include "MotorTask.h"
#include "Encoder.h"
#include "EncoderTask.h"
#include "UITask.h"
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

// Encoder telemetry shares (declare BEFORE tasks that reference them)
Share<float> g_encPosition{"enc_pos"};
Share<float>   g_encVelocity{"enc_vel"};
// Motor reference (0-100)
// Motor reference (-100..100)
Share<int8_t> g_vref{"vref"};
Share<int16_t> g_posref{"posref"};

// Shared command for encoder/motor (0=STOP,1=RUN,2=ZERO)
Share<int8_t> g_encCmd{"enc_cmd"};

// MotorTask/EncoderTask/UITask instances (pass pointers to the above shares/queue)
MotorTask motorTask(&motor, &g_encCmd, &g_encVelocity, &g_vref, &g_encPosition, &g_posref); 
EncoderTask encoderTask(&encoder, &g_encPosition, &g_encVelocity, &g_encCmd);
UITask uiTask(&g_encPosition, &g_encVelocity, &g_vref, &g_posref, &g_encCmd);

void setup() {
  Serial.begin(115200);
  delay(300);

  // Start Motor task (100 ms tick in its own entry function)
  xTaskCreate(
    motor_task_func,
    "MotorTask",
    2048,
    &motorTask,
    1,
    nullptr
  );
  // Start Encoder task (10 ms tick in its own entry function)
  xTaskCreate(
    encoder_task_func,
    "EncoderTask",
    2048,
    &encoderTask,
    2,
    nullptr
  );
  // Start UI task (200 ms tick in its own entry function)
  xTaskCreate(
    ui_task_func,
    "UITask",
    2048,
    &uiTask,
    3,
    nullptr
  );
 }

void loop() {
  // Idle; tasks run under FreeRTOS
  delay(600000000);
}
