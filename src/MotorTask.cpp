#include "MotorTask.h"
#include <Arduino.h>
// FreeRTOS APIs for task creation/delay
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Initialize static instance pointer
MotorTask* MotorTask::instance_ = nullptr;

MotorTask::MotorTask(DRV883* motor) noexcept
  : motor_(motor), fsm_(states_, 2)
{
  // Set instance for static wrappers
  instance_ = this;

  // Schedule initial wait
  wait_until_ms_ = millis() + MotorTask::WAIT_MS;
}

// ---------------------------------------------------------------------------
// FreeRTOS task entry
// This entry repeatedly calls MotorTask::update() at a fixed period.
extern "C" void motor_task_entry(void* arg) {
  MotorTask* mt = static_cast<MotorTask*>(arg);
  const TickType_t delayTicks = pdMS_TO_TICKS(10); // 10 ms tick
  for (;;) {
    if (mt) mt-> update();
    vTaskDelay(delayTicks);
  }
}

// WAIT execute: ensure motor stopped, if wait expired schedule run
int MotorTask::exec_wait() noexcept
{
    Serial.println("In wait exec");
  if (!instance_) return -1;
  if (instance_->motor_) instance_->motor_->stop();
  if (instance_->wait_until_ms_ == 0) {
    instance_->wait_until_ms_ = millis() + MotorTask::WAIT_MS;
    return MotorTask::WAIT;
  }

  if (millis() >= instance_->wait_until_ms_) {
    instance_->wait_until_ms_ = 0;
    instance_->run_until_ms_ = millis() + MotorTask::RUN_MS;
    return MotorTask::RUN;
  }

  return MotorTask::WAIT;
}

// RUN execute: drive motor; when done, stop and schedule wait
int MotorTask::exec_run() noexcept
{
    
  if (!instance_) return -1;

  // Ensure run is scheduled
  if (instance_->run_until_ms_ == 0) {
    instance_->run_until_ms_ = millis() + MotorTask::RUN_MS;
  }

  // Command motor
  if (instance_->motor_) 
  {
  instance_->motor_->setEff(100); // fixed effort; change as needed
  Serial.println("In run exec");
  }

  if (millis() >= instance_->run_until_ms_) {
    instance_->run_until_ms_ = 0;
    if (instance_->motor_) instance_->motor_->stop();
    instance_->wait_until_ms_ = millis() + MotorTask::WAIT_MS;
    return MotorTask::WAIT;
  }
  return MotorTask::RUN;
}
