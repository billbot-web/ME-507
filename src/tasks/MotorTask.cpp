/**
 * @file MotorTask.cpp
 * @brief Implementation of motor control task with velocity, position, and camera-based control modes
 * 
 * This file implements a comprehensive motor control system with multiple control modes:
 * - WAIT: Idle state with motor braked
 * - VRUN: Velocity control with feedforward and stiction compensation
 * - PRUN: Position control using PID with anti-windup
 * - CAMERA_PRUN: Camera error-based position control for visual tracking
 * 
 * Features include:
 * - Independent PID controllers for velocity and position control
 * - Tilt motor position clamping for safety (-150° to +10°)
 * - Stiction compensation to overcome static friction
 * - Dynamic state transitions based on command shares
 * 
 * @author Motor Control Development Team
 * @date December 2025
 * @version 3.0
 */

#include "MotorTask.h"
#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/// Static instance pointer for C-style callback functions
MotorTask* MotorTask::instance_ = nullptr;

/**
 * @brief Constructor for MotorTask
 * 
 * Initializes motor control task with all necessary shares and PID controllers.
 * PID controllers are externally managed to allow independent tuning per motor.
 * 
 * @param motor Pointer to DRV883 motor driver object
 * @param cmdShare Command share (0=WAIT, 1=VRUN, 2=PRUN, 3=CAMERA_PRUN)
 * @param veloShare Current velocity feedback share (deg/s)
 * @param vref Velocity reference share (desired velocity, deg/s)
 * @param posShare Current position feedback share (degrees)
 * @param posref Position reference share (desired position, degrees)
 * @param velocity_pid Pointer to velocity PID controller (externally managed)
 * @param position_pid Pointer to position PID controller (externally managed)
 * @param cameraErr Camera pixel error share for CAMERA_PRUN mode
 * @param isTiltMotor True if this is the tilt motor (enables position clamping)
 */
MotorTask::MotorTask(DRV883* motor,
                     Share<uint8_t>* cmdShare,
                     Share<float>* veloShare,
                     Share<int8_t>* vref, 
                     Share<float>* posShare,
                     Share<int16_t>* posref,
                     PID* velocity_pid,
                     PID* position_pid,
                     Share<int16_t>* cameraErr,
                     bool isTiltMotor) noexcept
  : motor_(motor),
    cmdShare_(cmdShare),
    veloShare_(veloShare),
    vref_(vref),
    posShare_(posShare),
    posref_(posref),
    velocity_pid_(velocity_pid),
    position_pid_(position_pid),
    cameraErr_(cameraErr),
    isTiltMotor_(isTiltMotor),
    fsm_(states_, 4)
{
  // Don't set instance_ here - will be set by task function
  // PID controllers are initialized externally and passed in

}
// ============================================================================
// State Machine Implementation
// ============================================================================

/**
 * @brief WAIT state - Motor idle and stopped
 * 
 * In this state, the motor is kept braked and waiting for commands.
 * Monitors the command share for state transition requests:
 * - Command 1: Transition to VRUN (velocity control)
 * - Command 2: Transition to PRUN (position control)  
 * - Command 3: Transition to CAMERA_PRUN (camera-based position control)
 * - Command 0 or other: Stay in WAIT
 * 
 * @return Next state ID to transition to
 * @retval WAIT Stay in wait state
 * @retval VRUN Transition to velocity control
 * @retval PRUN Transition to position control
 * @retval CAMERA_PRUN Transition to camera-based control
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
    else if (cmd == static_cast<int8_t>(3)) { // CAMERA_PRUN
      Serial.println("[MotorTask] Received CAMERA_PRUN command, transitioning to CAMERA_PRUN state");
      return static_cast<int>(CAMERA_PRUN);
    }
    // STOP/other -> stay in WAIT
  }
  return static_cast<int>(WAIT);
}

/**
 * @brief VRUN state - Velocity control with stiction compensation
 * 
 * Implements open-loop velocity control with the following features:
 * - Position-based safety limits for tilt motor (-150° to +10°)
 * - Directional clamping (only stops motion going further out of bounds)
 * - Stiction compensation to overcome static friction
 * - Dynamic state transitions based on command updates
 * 
 * Control Algorithm:
 * 1. Read desired velocity from vref share
 * 2. Check tilt motor position limits if applicable
 * 3. Add stiction compensation (30 for tilt, 0 for pan)
 * 4. Apply motor effort directly (open-loop control)
 * 
 * @return Next state ID to transition to
 * @retval VRUN Continue velocity control
 * @retval WAIT Stop motors
 * @retval PRUN Switch to position control
 * @retval CAMERA_PRUN Switch to camera-based control
 * 
 * @note Currently uses open-loop control (PID code commented out)
 * @note Stiction compensation adds baseline effort to overcome static friction
 */
uint8_t MotorTask::exec_velorun() noexcept {
  if (!instance_) return -1;

  // -------------------------------------------------------------------------
  // Step 1: Position Safety Checks for Tilt Motor
  // -------------------------------------------------------------------------
  
  // Get current position for clamping
  float current_position = 0.0f;
  if (instance_->posShare_) {
    current_position = instance_->posShare_->get();
  }

  // Get desired velocity for clamping check
  float desired_velocity = 0.0f;
  if (instance_->vref_) {
    desired_velocity = static_cast<float>(instance_->vref_->get());
  }

  // Check if this is the tilt motor and clamp position
  if (instance_->isTiltMotor_) {
    const float TILT_MIN = -150.0f;  ///< Minimum safe tilt angle (degrees)
    const float TILT_MAX = 10.0f;    ///< Maximum safe tilt angle (degrees)
    
    // Only stop if moving FURTHER out of bounds (allows recovery movement)
    if (current_position < TILT_MIN && desired_velocity < 0) {
      // Past min limit and trying to go more negative - STOP
      Serial.println("[MotorTask] Tilt at MIN limit, blocking negative motion");
      instance_->motor_->brake();
      if (instance_->cmdShare_) instance_->cmdShare_->put(0);
    } else if (current_position > TILT_MAX && desired_velocity > 0) {
      // Past max limit and trying to go more positive - STOP
      Serial.println("[MotorTask] Tilt at MAX limit, blocking positive motion");
      instance_->motor_->brake();
      if (instance_->cmdShare_) instance_->cmdShare_->put(0);
    }
    // If past limit but moving back toward safe zone, allow it
  }

  // -------------------------------------------------------------------------
  // Step 2: Check for State Transition Commands
  // -------------------------------------------------------------------------
  
  // Pull latest mode each tick to allow immediate stop
  if (instance_->cmdShare_) {
    int8_t new_state = instance_->cmdShare_->get();
    if (new_state == static_cast<int8_t>(0)) { // STOP
      instance_->motor_->brake();
      Serial.print("[MotorTask] Velocity run switching to wait");
      return static_cast<int>(WAIT);
    }
    if (new_state == static_cast<int8_t>(2)) { // POSITION_RUN
      Serial.println("[MotorTask] Velocity run switching to position run");
      return static_cast<int>(PRUN);
    }
    if (new_state == static_cast<int8_t>(3)) { // CAMERA_PRUN
      Serial.println("[MotorTask] Velocity run switching to camera position run");
      return static_cast<int>(CAMERA_PRUN);
    }
    // otherwise continue running
  }
  
  // -------------------------------------------------------------------------
  // Step 3: Velocity Control (Currently Open-Loop)
  // -------------------------------------------------------------------------
  
  // NOTE: PID velocity control is commented out below
  // Currently using direct feedforward control for simplicity
  
  // // velocity PID Control
  // float current_velocity = 0.0f;  // Current velocity
  
  // // Get current velocity from velocity share
  // if (instance_->veloShare_) {
  //   current_velocity = instance_->veloShare_->get();
  // }
  
  // // Calculate velocity error (setpoint - current)
  // float velocity_error = desired_velocity - current_velocity;
  
  // // Update PID with velocity error
  // instance_->velocity_pid_->UpdateError(velocity_error);
  
  // // Get PID output (correction term)
  // double pid_output = instance_->velocity_pid_->TotalError();

  // Convert velocity (deg/s) to effort (%)
  // Tune this gain based on your motor's speed-to-effort relationship
  //const float VELOCITY_TO_EFFORT_GAIN = 0.125f;  // Adjust as needed
  //float feedforward = desired_velocity * VELOCITY_TO_EFFORT_GAIN;
  
  // -------------------------------------------------------------------------
  // Step 4: Stiction Compensation
  // -------------------------------------------------------------------------
  
  // Anti-stiction: add minimum effort to overcome static friction when moving
  int STICTION_COMPENSATION;
  if (instance_->isTiltMotor_) {
    STICTION_COMPENSATION = 30;  ///< Tilt motor stiction compensation (%)
  } else {
    STICTION_COMPENSATION = 0;   ///< Pan motor stiction compensation (%)
  }
  
  // Add stiction compensation when velocity command is above threshold
  if (desired_velocity > 0.5f) {
    desired_velocity += STICTION_COMPENSATION;
  } else if (desired_velocity < -0.5f) {
    desired_velocity -= STICTION_COMPENSATION;
  }
  
  // -------------------------------------------------------------------------
  // Step 5: Apply Motor Effort
  // -------------------------------------------------------------------------
  
  // Apply motor effort
  if (instance_->motor_) {
    instance_->motor_->setEff(desired_velocity);
    Serial.println(desired_velocity);
  }
  
  // Store last effort for debugging/continuity
  instance_->last_effort_ = desired_velocity;
  
  return static_cast<int>(VRUN);
}

/**
 * @brief PRUN state - Position control using PID with stiction compensation
 * 
 * Implements closed-loop position control with the following features:
 * - PID control with anti-windup (rolling window of 25 samples)
 * - Position-based safety limits for tilt motor (-150° to +10°)
 * - Directional clamping (only stops motion going further out of bounds)
 * - Stiction compensation with deadband to prevent oscillation near target
 * - Motor effort constrained to ±65% for safety
 * 
 * Control Algorithm:
 * 1. Read desired and current positions
 * 2. Check tilt motor position limits if applicable
 * 3. Calculate position error (setpoint - current)
 * 4. Update PID controller with error
 * 5. Add stiction compensation when far from target
 * 6. Constrain effort and apply to motor
 * 
 * @return Next state ID to transition to
 * @retval PRUN Continue position control
 * @retval WAIT Stop motors
 * @retval VRUN Switch to velocity control
 * @retval CAMERA_PRUN Switch to camera-based control
 * 
 * @note PID gains are externally configured (typically Kp=0.08, Ki=0.0005)
 * @note Stiction compensation: 35% for tilt, 5% for pan
 * @note Deadband: 5° to prevent oscillation near target
 */
uint8_t MotorTask::exec_posrun() noexcept {
  if (!instance_) return -1;

  // -------------------------------------------------------------------------
  // Step 1: Read Position Feedback and Reference
  // -------------------------------------------------------------------------
  
  // Position PID Control
  float desired_position = 0.0f;
  float current_position = 0.0f;
  
  // Get desired position from posref share
  if (instance_->posref_) {
    desired_position = static_cast<float>(instance_->posref_->get());
  }
  // Get current position from position share
  if (instance_->posShare_) {
    current_position = instance_->posShare_->get();
  }

  // -------------------------------------------------------------------------
  // Step 2: Position Safety Checks for Tilt Motor
  // -------------------------------------------------------------------------
  
  // Check if this is the tilt motor and clamp position
  if (instance_->isTiltMotor_) {
    const float TILT_MIN = -150.0f;  ///< Minimum safe tilt angle (degrees)
    const float TILT_MAX = 10.0f;    ///< Maximum safe tilt angle (degrees)
    
    // Calculate which way we're trying to move
    float position_error = desired_position - current_position;
    
    // Only stop if moving FURTHER out of bounds (allows recovery movement)
    if (current_position < TILT_MIN && position_error < 0) {
      // Past min limit and trying to go more negative - STOP
      Serial.println("[MotorTask] Tilt at MIN limit, blocking negative motion");
      instance_->motor_->brake();
      if (instance_->cmdShare_) instance_->cmdShare_->put(0);
    } else if (current_position > TILT_MAX && position_error > 0) {
      // Past max limit and trying to go more positive - STOP
      Serial.println("[MotorTask] Tilt at MAX limit, blocking positive motion");
      instance_->motor_->brake();
      if (instance_->cmdShare_) instance_->cmdShare_->put(0);
    }
    // If past limit but moving back toward safe zone, allow it
  }

  // -------------------------------------------------------------------------
  // Step 3: Check for State Transition Commands
  // -------------------------------------------------------------------------
  
  // Pull latest mode each tick to allow immediate stop
  if (instance_->cmdShare_) {
    uint8_t new_state = instance_->cmdShare_->get();
    if (new_state == static_cast<uint8_t>(0)) { // STOP
      Serial.println("[MotorTask] Position run received STOP, returning to WAIT");
      return static_cast<int>(WAIT);
    } else if (new_state == static_cast<uint8_t>(1)) { // VELOCITY_RUN
      Serial.println("[MotorTask] Position run switching to velocity run");
      return static_cast<int>(VRUN);
    } else if (new_state == static_cast<uint8_t>(3)) { // CAMERA_PRUN
      Serial.println("[MotorTask] Position run switching to camera position run");
      return static_cast<int>(CAMERA_PRUN);
    }
    // otherwise continue running in position mode
  }

  // -------------------------------------------------------------------------
  // Step 4: PID Control Computation
  // -------------------------------------------------------------------------
  
  // Calculate position error (setpoint - current)
  float position_error = desired_position - current_position;
 
  // Update PID with position error
  instance_->position_pid_->UpdateError(position_error);
  
  // Get PID output (motor effort)
  double pid_output = instance_->position_pid_->TotalError();

  // -------------------------------------------------------------------------
  // Step 5: Stiction Compensation
  // -------------------------------------------------------------------------
  
  // Anti-stiction: add feedforward to overcome static friction
  // BUT only when far from target to avoid oscillation near setpoint
  int STICTION_COMPENSATION;
  if (instance_->isTiltMotor_) {
    STICTION_COMPENSATION = 35;  ///< Tilt motor stiction compensation (%)
  } else {
    STICTION_COMPENSATION = 5;   ///< Pan motor stiction compensation (%)
  }
  const float STICTION_DEADBAND = 5.0f;  ///< Degrees - disable stiction near target
  int feedforward = 0;
  
  // Only apply stiction compensation when far from target
  if (fabs(position_error) > STICTION_DEADBAND) {
    if (pid_output > 0.50f) {
      feedforward = STICTION_COMPENSATION;
    } else if (pid_output < -0.50f) {
      feedforward = -STICTION_COMPENSATION;
    }
  }
  
  // -------------------------------------------------------------------------
  // Step 6: Apply Motor Effort with Safety Limits
  // -------------------------------------------------------------------------
  
  // Convert to motor effort range (-100 to 100) and add feedforward
  int motor_effort = static_cast<int>(pid_output) + feedforward;
  
  // Constrain motor effort to ±65 for safety
  motor_effort = constrain(motor_effort, -65, 65);
  
  // Apply motor effort
  if (instance_->motor_) {
    instance_->motor_->setEff(motor_effort);
  }
  // Store last effort for debugging/continuity
  instance_->last_effort_ = motor_effort;

  return static_cast<int>(PRUN);
}

/**
 * @brief CAMERA_PRUN state - Camera pixel error-based position control
 * 
 * Implements visual servoing using camera pixel error as the control signal.
 * This mode enables direct visual tracking without explicit position setpoints.
 * 
 * Key differences from PRUN mode:
 * - Error comes directly from camera (pixels) instead of position difference
 * - Error is scaled by 2x for PID tuning
 * - Uses same PID controller as position mode
 * - Includes same safety features (position limits, stiction compensation)
 * 
 * Control Algorithm:
 * 1. Read camera pixel error from cameraErr share
 * 2. Check tilt motor position limits if applicable
 * 3. Scale error by 2.0 and update PID controller
 * 4. Add stiction compensation when far from target
 * 5. Constrain effort and apply to motor
 * 
 * @return Next state ID to transition to
 * @retval CAMERA_PRUN Continue camera-based control
 * @retval WAIT Stop motors
 * @retval VRUN Switch to velocity control
 * @retval PRUN Switch to position control
 * 
 * @note Camera error is in pixels (center of frame = 0)
 * @note Error is scaled by 2.0 to match PID tuning from position mode
 * @note Shares same PID controller gains as PRUN mode
 */
uint8_t MotorTask::exec_camera_posrun() noexcept {
  if (!instance_) return -1;

  // -------------------------------------------------------------------------
  // Step 1: Read Camera Error and Current Position
  // -------------------------------------------------------------------------
  
  // Get current position for clamping
  float current_position = 0.0f;
  if (instance_->posShare_) {
    current_position = instance_->posShare_->get();
  }

  // Get camera error (pixels) - this is our position error
  float position_error = 0.0f;
  if (instance_->cameraErr_) {
    position_error = static_cast<float>(instance_->cameraErr_->get());
  }

  // -------------------------------------------------------------------------
  // Step 2: Position Safety Checks for Tilt Motor
  // -------------------------------------------------------------------------
  
  // Check if this is the tilt motor and clamp position
  if (instance_->isTiltMotor_) {
    const float TILT_MIN = -150.0f;  ///< Minimum safe tilt angle (degrees)
    const float TILT_MAX = 10.0f;    ///< Maximum safe tilt angle (degrees)
    
    // Only stop if moving FURTHER out of bounds (allows recovery movement)
    if (current_position < TILT_MIN && position_error < 0) {
      Serial.println("[MotorTask] Tilt at MIN limit, blocking negative motion");
      instance_->motor_->brake();
      if (instance_->cmdShare_) instance_->cmdShare_->put(0);
    } else if (current_position > TILT_MAX && position_error > 0) {
      Serial.println("[MotorTask] Tilt at MAX limit, blocking positive motion");
      instance_->motor_->brake();
      if (instance_->cmdShare_) instance_->cmdShare_->put(0);
    }
  }

  // -------------------------------------------------------------------------
  // Step 3: Check for State Transition Commands
  // -------------------------------------------------------------------------
  
  // Pull latest mode each tick to allow immediate stop
  if (instance_->cmdShare_) {
    uint8_t new_state = instance_->cmdShare_->get();
    if (new_state == static_cast<uint8_t>(0)) { // STOP
      Serial.println("[MotorTask] Camera position run received STOP, returning to WAIT");
      return static_cast<int>(WAIT);
    } else if (new_state == static_cast<uint8_t>(1)) { // VELOCITY_RUN
      Serial.println("[MotorTask] Camera position run switching to velocity run");
      return static_cast<int>(VRUN);
    } else if (new_state == static_cast<uint8_t>(2)) { // POSITION_RUN
      Serial.println("[MotorTask] Camera position run switching to position run");
      return static_cast<int>(PRUN);
    }
    // otherwise continue running in camera position mode
  }
   if (instance_->cameraErr_) {
        Serial.print("Camera Error: ");
        Serial.println(instance_->cameraErr_->get());
        int16_t cam_error = instance_->cameraErr_->get();
    }

  // -------------------------------------------------------------------------
  // Step 4: PID Control Computation with Camera Error
  // -------------------------------------------------------------------------

  // Update PID with camera error (scaled by 1.0x for tuning)
  // Note: Unlike PRUN mode, error comes directly from camera (no desired-current calc)
  instance_->position_pid_->UpdateError(position_error*1.0f);
  
  // Get PID output (motor effort)
  double pid_output = instance_->position_pid_->TotalError();

  // -------------------------------------------------------------------------
  // Step 5: Stiction Compensation
  // -------------------------------------------------------------------------
  
  // Anti-stiction: add feedforward to overcome static friction
  // BUT only when far from target to avoid oscillation near zero error
  int STICTION_COMPENSATION;
  if (instance_->isTiltMotor_) {
    STICTION_COMPENSATION = 35;  ///< Tilt motor stiction compensation (%)
  } else {
    STICTION_COMPENSATION = 5;   ///< Pan motor stiction compensation (%)
  }
  const float STICTION_DEADBAND = 5.0f;  ///< Pixels - disable stiction near target
  int feedforward = 0;
  
  // Only apply stiction compensation when far from target
  if (fabs(position_error) > STICTION_DEADBAND) {
    if (pid_output > 0.50f) {
      feedforward = STICTION_COMPENSATION;
    } else if (pid_output < -0.50f) {
      feedforward = -STICTION_COMPENSATION;
    }
  }
  
  // -------------------------------------------------------------------------
  // Step 6: Apply Motor Effort with Safety Limits
  // -------------------------------------------------------------------------
  
  // Convert to motor effort range (-100 to 100) and add feedforward
  int motor_effort = static_cast<int>(pid_output) + feedforward;
  
  // Constrain motor effort to ±65 for safety
  motor_effort = constrain(motor_effort, -65, 65);
  
  // Apply motor effort
  if (instance_->motor_) {
    instance_->motor_->setEff(motor_effort);
  }
  // Store last effort for debugging/continuity
  instance_->last_effort_ = motor_effort;

  return static_cast<int>(CAMERA_PRUN);
}

