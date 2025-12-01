/**
 * @file EncoderTask.cpp
 * @brief Implementation of EncoderTask: a FreeRTOS-based task with 5-state FSM
 *        States: CHOOSE_MODE, CALIBRATE, MOTOR_TEST, TELEOP, TRACKER
 */
// Keep includes minimal; FSM/State are required for the task FSM
#include <Arduino.h>
#include "FSM.h"
#include "State.h"
#include "EncoderTask.h"
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
  instance_ = this;
  if (encoder_) (void)encoder_->begin();
}
/**
 * @brief FreeRTOS task entry; repeatedly calls update() at configured period.
 */
// ---------------------------------------------------------------------------
// FreeRTOS task entry
// This entry repeatedly calls EncoderTask::update() at a fixed period.
  extern "C" void encoder_task_func(void* arg) {
  EncoderTask* encoderTask = static_cast<EncoderTask*>(arg);
  const TickType_t delayTicks = pdMS_TO_TICKS(10); // 10 ms tick
  for (;;) {
    if (encoderTask) {
      EncoderTask::instance_ = encoderTask;  // Set instance for this task's context
      encoderTask->update();
    }
    vTaskDelay(delayTicks);
  }
}
// ---------------- State implementations ----------------

/**
 * @brief WAIT state: idle, handle ZERO/START commands.
 * @return Next state id (WAIT or RUN)
 */
uint8_t EncoderTask::exec_wait() noexcept
{
  if (!instance_) return -1;

  static uint32_t lastDebug = 0;
  if (millis() - lastDebug > 3000) {
    Serial.println("[EncoderTask] In WAIT state");
    if (instance_->cmdShare_) {
      Serial.print("[EncoderTask] cmdShare value: ");
      Serial.println(instance_->cmdShare_->get());
    }
    lastDebug = millis();
  }

  // Check for zero command
  if (instance_->zeroShare_ && instance_->zeroShare_->get()) {
    if (instance_->encoder_) instance_->encoder_->zero();
    if (instance_->positionShare_) instance_->positionShare_->put(0.0f);
    if (instance_->velocityShare_) instance_->velocityShare_->put(0.0f);
    // Clear the zero flag
    instance_->zeroShare_->put(false);
    Serial.println("[EncoderTask] Encoder zeroed");
  }

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
 * @brief RUN state: read encoder, publish shares, honor STOP/ZERO.
 * @return Next state id (RUN or WAIT)
 */
uint8_t EncoderTask::exec_run() noexcept
{
  if (!instance_) return -1;

  static uint32_t lastDebug = 0;
  if (millis() - lastDebug > 3000) {
    Serial.println("[EncoderTask] In RUN state, reading encoder");
    lastDebug = millis();
  }

  // Check for zero command
  if (instance_->zeroShare_ && instance_->zeroShare_->get()) {
    if (instance_->encoder_) instance_->encoder_->zero();
    if (instance_->positionShare_) instance_->positionShare_->put(0.0f);
    if (instance_->velocityShare_) instance_->velocityShare_->put(0.0f);
    // Clear the zero flag
    instance_->zeroShare_->put(false);
    Serial.println("[EncoderTask] Encoder zeroed");
  }

  // Handle any pending commands without blocking
  //The commands are:
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

  // Sample encoder and publish
  if (instance_->encoder_) {
    instance_->encoder_->update();
    const float pos = instance_->encoder_->get_position();
    //degrees/second
    const float vel_cps = static_cast<float>(instance_->encoder_->get_velocity() * 1e6); // degrees/second

    static uint32_t lastPrint = 0;
    if (millis() - lastPrint > 3000) {
      Serial.print("[EncoderTask] Raw position: ");
      Serial.print(pos);
      Serial.print(", velocity: ");
      Serial.println(vel_cps);
      lastPrint = millis();
    }

    if (instance_->positionShare_) instance_->positionShare_->put(pos);
    if (instance_->velocityShare_) instance_->velocityShare_->put(vel_cps);
  }

  return static_cast<int>(RUN);
}
