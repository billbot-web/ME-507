#include "MotorTask.h"
#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

MotorTask* MotorTask::instance_ = nullptr;

MotorTask::MotorTask(DRV883* motor,
                     Share<uint8_t>* cmdShare,
                     Share<float>* veloShare,
                     Share<int8_t>* vref, 
                     Share<float>* posShare,
                     Share<int16_t>* posref) noexcept
  : motor_(motor),
    cmdShare_(cmdShare),
    veloShare_(veloShare),
    vref_(vref),
    posShare_(posShare),
    posref_(posref),
    // PID Controllers: Kp, Ki, Kd 
    velocity_pid_(),
    position_pid_(),
    fsm_(states_, 3)
{
  instance_ = this;
  // Initialize PID controllers with gains
  // Kp, Ki, Kd
  velocity_pid_.Init(1.0, 0.0, 0.0);
  position_pid_.Init(0.80, 0.05, 0.0);
}
// ---------------- State implementations ----------------
/**
 * @brief WAIT state: idle, handle STOP/START commands.
 * @return Next state id (WAIT or RUN)
 */
uint8_t MotorTask::exec_wait() noexcept {
  if (!instance_) return -1;

  // Always keep motor stopped while waiting
  if (instance_->motor_) instance_->motor_->brake();

  // Read latest command from the shared command variable (if present).
  if (instance_->cmdShare_) {
    uint8_t cmd = instance_->cmdShare_->get();
    if (cmd == static_cast<int8_t>(1)) { // VELOCITY_RUN
      Serial.println("[MotorTask] Received VELOCITY_RUN command, transitioning to VRUN state");
      return static_cast<int>(VRUN);  
    }  
    else if (cmd == static_cast<int8_t>(2)) { // POSITION_RUN
      Serial.println("[MotorTask] Received POSITION_RUN command, transitioning to PRUN state");
      return static_cast<int>(PRUN);
    }
    // STOP/other -> stay in WAIT
  }
  return static_cast<int>(WAIT);
}

/**
 * @brief RUN state: apply motor effort from vref share; handle STOP command.
 * @return Next state id (WAIT or RUN)
 */
uint8_t MotorTask::exec_velorun() noexcept {
  if (!instance_) return -1;

  // Pull latest mode each tick to allow immediate stop
  if (instance_->cmdShare_) {
    int8_t new_state = instance_->cmdShare_->get();
    if (new_state == static_cast<int8_t>(0)) { // STOP
      instance_->motor_->brake();
      return static_cast<int>(WAIT);
    }
    if (new_state == static_cast<int8_t>(2)) { // POSITION_RUN
      Serial.println("[MotorTask] Velocity run switching to position run");
      return static_cast<int>(PRUN);
    }
    // otherwise continue running
  }
  
  // Position PID Control
  float desired_velocity = 0.0f;  // Default setpoint
  float current_velocity = 0.0f;  // Current position
  
  // Get desired position from posref share
  if (instance_->posref_) {
    desired_velocity = static_cast<float>(instance_->vref_->get());
  }
  // Get current position from position share
  if (instance_->posShare_) {
    current_velocity = instance_->posShare_->get();
  }
  
  // Calculate position error (setpoint - current)
  float velocity_error = desired_velocity - current_velocity;
 
  
  // Update PID with position error
  instance_->position_pid_.UpdateError(velocity_error);
  
  // Get PID output (motor effort)
  double pid_output = instance_->position_pid_.TotalError();
  
  // Convert to motor effort range (-100 to 100)
  int motor_effort = static_cast<int>(pid_output);
  
  // Apply motor effort
  if (instance_->motor_) {
    instance_->motor_->setEff(motor_effort);
  }
  // Store last effort for debugging/continuity
  instance_->last_effort_ = motor_effort;

  return static_cast<int>(VRUN);
}

/**
 * @brief RUN state: apply motor effort from vref share; handle STOP command.
 * @return Next state id (WAIT or RUN)
 */
uint8_t MotorTask::exec_posrun() noexcept {
  if (!instance_) return -1;

  // Pull latest mode each tick to allow immediate stop
  if (instance_->cmdShare_) {
    uint8_t new_state = instance_->cmdShare_->get();
    if (new_state == static_cast<uint8_t>(0)) { // STOP
      Serial.println("[MotorTask] Position run received STOP, returning to WAIT");
      return static_cast<int>(WAIT);
    } else if (new_state == static_cast<uint8_t>(1)) { // VELOCITY_RUN
      Serial.println("[MotorTask] Position run switching to velocity run");
      return static_cast<int>(VRUN);
    }
    // otherwise continue running in position mode
  }

  // Position PID Control
  float desired_position = 0.0f;  // Default setpoint
  float current_position = 0.0f;  // Current position
  
  // Get desired position from posref share
  if (instance_->posref_) {
    desired_position = static_cast<float>(instance_->posref_->get());
  }
  // Get current position from position share
  if (instance_->posShare_) {
    current_position = instance_->posShare_->get();
  }
  
  // Calculate position error (setpoint - current)
  float position_error = desired_position - current_position;
 
  
  // Update PID with position error
  instance_->position_pid_.UpdateError(position_error);
  
  // Get PID output (motor effort)
  double pid_output = instance_->position_pid_.TotalError();
  
  // Convert to motor effort range (-100 to 100)
  int motor_effort = static_cast<int>(pid_output);
  
  // Apply motor effort
  if (instance_->motor_) {
    instance_->motor_->setEff(motor_effort);
  }
  // Store last effort for debugging/continuity
  instance_->last_effort_ = motor_effort;

  return static_cast<int>(PRUN);
}

