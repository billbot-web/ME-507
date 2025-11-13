#include "MotorTask.h"
#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

MotorTask* MotorTask::instance_ = nullptr;

MotorTask::MotorTask(DRV883* motor,
                     Share<int8_t>* cmdShare,
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
    fsm_(states_, 3)
{
  instance_ = this;
}

/**
 * @brief FreeRTOS task entry; repeatedly calls update() at configured period.
 */
// ---------------------------------------------------------------------------
// FreeRTOS task entry
// This entry repeatedly calls MotorTask::update() at a fixed period.
  extern "C" void motor_task_func(void* arg) {
  MotorTask* motorTask = static_cast<MotorTask*>(arg);
  const TickType_t delayTicks = pdMS_TO_TICKS(100); // 100 ms tick
  for (;;) {
    if (motorTask) motorTask-> update();
    vTaskDelay(delayTicks);
  }
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
    int8_t cmd = instance_->cmdShare_->get();
    if (cmd == static_cast<int8_t>(1)) { // RUN
      return static_cast<int>(VRUN);  
    }  
    else if (cmd == static_cast<int8_t>(2)) { // PRUN
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
      return static_cast<int>(WAIT);
    }
    // otherwise continue running
  }

  {
    // Read signed vref (-100 .. +100) and apply to motor (supports forward/reverse)
    int vref = instance_->last_effort_; // Preserve last effort initially
    if (instance_->vref_) {
      vref = static_cast<int>(instance_->vref_->get());
    }
    // Clamp to allowed DRV883 range
    if (vref > 100) vref = 100;
    if (vref < -100) vref = -100;
    if (instance_->motor_) {
      instance_->motor_->setEff(vref);
    }
  }

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
    int8_t new_state = instance_->cmdShare_->get();
    if (new_state == static_cast<int8_t>(0)) { // STOP
      return static_cast<int>(WAIT);
    }
    // otherwise continue running
  }


      // Read signed vref (-100 .. +100) and apply to motor (supports forward/reverse)
  int vref = instance_->last_effort_; // Preserve last effort initially
  if (instance_->vref_) {
    vref = static_cast<int>(instance_->vref_->get());
  }
  // Clamp to allowed DRV883 range
  if (vref > 100) vref = 100;
  if (vref < -100) vref = -100;
  if (instance_->motor_) {
    instance_->motor_->setEff(vref);
 }

  return static_cast<int>(PRUN);
}

