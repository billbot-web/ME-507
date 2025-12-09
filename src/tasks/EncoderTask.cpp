/**
 * @file EncoderTask.cpp
 * @brief Implementation of high-frequency encoder monitoring with velocity calculation
 * 
 * This file implements the EncoderTask which provides continuous position and velocity
 * measurement using the ESP32 PCNT peripheral. The task operates at high frequency
 * (default 100Hz) to capture encoder changes with minimal latency and computes
 * velocity using the Encoder class's sliding window smoothing algorithm.
 * 
 * Operating Modes:
 * - **WAIT**: Encoder monitoring paused, position held constant
 * - **RUN**: Active monitoring with continuous position/velocity updates
 * 
 * Key Operations:
 * - Calls encoder_->update() each cycle to refresh hardware counter
 * - Reads position and velocity from Encoder object
 * - Converts counts to degrees using gear ratio (default 70:1 gearbox × 64 CPR)
 * - Publishes data via Share<T> for thread-safe access by control loops
 * - Supports zero/home operation to reset position accumulator
 * 
 * Data Output:
 * - Position: Accumulated angle in degrees from last zero
 * - Velocity: Angular rate in degrees/second (smoothed)
 * 
 * Performance:
 * - High update rate (10ms default) minimizes control loop latency
 * - Velocity smoothing reduces measurement noise
 * - Hardware PCNT ensures no missed counts even at high shaft speeds
 * 
 * @author Encoder Control Team
 * @date December 2025
 * @version 1.6
 * 
 * @see EncoderTask.h for class interface and configuration options
 * @see Encoder.h for PCNT hardware interface details
 */
// Keep includes minimal; FSM/State are required for the task FSM
#include <Arduino.h>
#include "../fsm/FSM.h"
#include "../fsm/State.h"
#include "EncoderTask.h"
#include "UITask.h"  // For accessing telemetry queues
// Need FreeRTOS queue prototype for xQueueReceive when accessing raw handle
#include "freertos/queue.h"

// Static instance used by state wrappers
EncoderTask* EncoderTask::instance_ = nullptr;

/**
 * @brief Construct the Encoder task and bind shares/queue.
 * @param encoder        Pointer to Encoder (not owned)
 * @param positionShare  Share to publish position (counts)
 * @param velocityShare  Share to publish velocity (counts/sec)
 * @param cmdShare       Share to receive EncoderTask::Command
 * @param zeroShare      Share to receive zero command (boolean)
 * @param ui_mode        Share for UI mode (0=choose, 1=tracker, 2=teleop)
 * @param dcalibrate     Share for calibration flag
 * @param updateMs       Period in ms between updates
 */
EncoderTask::EncoderTask(Encoder* encoder,
                         Share<float>* positionShare,
                         Share<float>* velocityShare,
                         Share<uint8_t>* cmdShare,
                         Share<bool>* zeroShare,
                         uint32_t updateMs) noexcept
  : encoder_(encoder),
    updateMs_(updateMs),
    positionShare_(positionShare),
    velocityShare_(velocityShare),
    cmdShare_(cmdShare),
    zeroShare_(zeroShare),
    fsm_(states_, 2)
{
  // Don't set instance_ here - will be set by task function
  if (encoder_) (void)encoder_->begin();
}

// ---------------- State implementations ----------------

/**
 * @brief WAIT state - Encoder monitoring paused, awaiting commands
 * 
 * In this state, the encoder hardware counter continues running but position/velocity
 * are not being updated or published. The task monitors for commands to transition
 * to RUN state or to zero the position accumulator.
 * 
 * State Behavior:
 * 1. Check for zero command from zeroShare
 * 2. If zero requested: Reset encoder position to 0, publish zeros, clear flag
 * 3. Check cmdShare for state transition commands
 * 4. If command = 1 (VRUN) or 2 (PRUN): Transition to RUN state
 * 5. Otherwise: Stay in WAIT
 * 
 * Commands:
 * - cmdShare = 0: Stay in WAIT (motor stopped)
 * - cmdShare = 1: Transition to RUN (velocity control active)
 * - cmdShare = 2: Transition to RUN (position control active)
 * - zeroShare = true: Reset position accumulator to zero (home)
 * 
 * @return Next state ID
 * @retval WAIT Continue waiting (no run command)
 * @retval RUN Begin encoder monitoring (run command received)
 * 
 * @note Zero operation clears position but does not lose encoder counts
 * @note Position is published as 0.0 after zeroing for immediate feedback
 */
uint8_t EncoderTask::exec_wait() noexcept
{
  if (!instance_) return -1;

  static uint32_t lastDebug = 0;

  // -------------------------------------------------------------------------
  // Step 1: Check for Zero/Home Command
  // -------------------------------------------------------------------------
  
  // Check for zero command
  if (instance_->zeroShare_ && instance_->zeroShare_->get()) {
    if (instance_->encoder_) instance_->encoder_->zero();
    if (instance_->positionShare_) instance_->positionShare_->put(0.0f);
    if (instance_->velocityShare_) instance_->velocityShare_->put(0.0f);
    // Clear the zero flag
    instance_->zeroShare_->put(false);
    Serial.println("[EncoderTask] Encoder zeroed");
  }

  // -------------------------------------------------------------------------
  // Step 2: Check for State Transition Commands
  // -------------------------------------------------------------------------

  // Read latest command from the shared command variable (if present)
  if (instance_->cmdShare_) {
    uint8_t raw_cmd = instance_->cmdShare_->get();
    switch (raw_cmd) {
      case 1: // VeloRUN
        Serial.println("[EncoderTask] Transitioning to RUN state");
        return static_cast<int>(RUN);
      case 2: // PosRUN 
        Serial.println("[EncoderTask] Transitioning to RUN state");
        return static_cast<int>(RUN);
      case 0: // STOP
      default:
        break; // already stopped
    }
  }

  return static_cast<int>(WAIT);
}

/**
 * @brief RUN state - Continuous encoder monitoring with position/velocity publication
 * 
 * This is the primary operating state where encoder position and velocity are
 * continuously measured and published. The task reads the hardware PCNT counter,
 * calculates velocity using the Encoder's sliding window algorithm, converts to
 * degrees, and publishes via Share<T> for control loop access.
 * 
 * State Behavior:
 * 1. Check for zero command (allows homing during operation)
 * 2. Check for stop command (transition to WAIT)
 * 3. Call encoder_->update() to refresh position and velocity
 * 4. Read position (degrees) and velocity (degrees/sec)
 * 5. Publish data via positionShare_ and velocityShare_
 * 6. Push to telemetry queues for UI display
 * 7. Stay in RUN state
 * 
 * Commands:
 * - cmdShare = 0: Transition to WAIT (stop monitoring)
 * - cmdShare = 1 or 2: Continue in RUN (motor control active)
 * - zeroShare = true: Reset position to zero without stopping
 * 
 * Data Conversion:
 * - Raw encoder counts → degrees (using gear ratio from Encoder class)
 * - Velocity: counts/μs → degrees/second (multiply by 1e6)
 * 
 * @return Next state ID
 * @retval RUN Continue monitoring (normal operation)
 * @retval WAIT Stop monitoring (stop command received)
 * 
 * @note Position and velocity smoothing handled by Encoder class
 * @note Telemetry queues are non-blocking (old values overwritten)
 * @note Called at high frequency (default 100Hz) for minimal latency
 */
uint8_t EncoderTask::exec_run() noexcept
{
  if (!instance_) return -1;
  
  // -------------------------------------------------------------------------
  // Step 1: Check for Zero/Home Command
  // -------------------------------------------------------------------------
  
  // Check for zero command
  if (instance_->zeroShare_ && instance_->zeroShare_->get()) {
    if (instance_->encoder_) instance_->encoder_->zero();
    if (instance_->positionShare_) instance_->positionShare_->put(0.0f);
    if (instance_->velocityShare_) instance_->velocityShare_->put(0.0f);
    // Clear the zero flag
    instance_->zeroShare_->put(false);
    Serial.println("[EncoderTask] Encoder zeroed");
  }

  // -------------------------------------------------------------------------
  // Step 2: Check for State Transition Commands
  // -------------------------------------------------------------------------
  
  // Handle any pending commands without blocking
  // The commands are:
  // 0: STOP
  // 1: VELOCITY_RUN
  // 2: POSITION_RUN
  int8_t raw = 0;
  if (instance_->cmdShare_) {
    raw = instance_->cmdShare_->get();
    switch (raw) {
      case 0: // STOP
        return static_cast<int>(WAIT);
      case 1: // VELOCITY_RUN
      case 2: // POSITION_RUN (let it continue running, don't interfere)
      default:
        break; // already running
    }
  }

  // -------------------------------------------------------------------------
  // Step 3: Read Encoder and Publish Data
  // -------------------------------------------------------------------------
  
  // Sample encoder and publish
  if (instance_->encoder_) {
    instance_->encoder_->update();
    const float pos = instance_->encoder_->get_position();
    //degrees/second
    const float vel_cps = static_cast<float>(instance_->encoder_->get_velocity() * 1e6); // degrees/second

    if (instance_->positionShare_) instance_->positionShare_->put(pos);
    if (instance_->velocityShare_) instance_->velocityShare_->put(vel_cps);
    
    // Push to telemetry queues (non-blocking, overwrite old values)
    if (instance_->vel_queue_) instance_->vel_queue_->put(vel_cps);
    if (instance_->pos_queue_) instance_->pos_queue_->put(pos);
  }

  return static_cast<int>(RUN);
}
