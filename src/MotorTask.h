#pragma once
#include "FSM.h"
#include "DRV883.h"

/**
 * @file MotorTask.h
 * @brief Simple motor task with two states: WAIT and RUN.
 *
 * This class encapsulates motor control logic (start/stop timing)
 * using the project's `State`/`FSM` pattern. Because `State` expects a plain C
 * function pointer for the execute function, `MotorTask` uses static wrapper
 * functions and a static `instance_` pointer to bridge to instance data.
 */
class MotorTask {
public:
  enum Ids { WAIT = 0, RUN = 1 };

  // Accept a pointer to the motor driver (caller retains ownership).
  MotorTask(DRV883* motor) noexcept;

  /// Run one FSM tick: scan states and execute the selected state's behavior
  void update() noexcept { fsm_.run_curstate(); }

private:
  // Static wrappers used for State C-style function pointers
  static int exec_wait() noexcept;
  static int exec_run() noexcept;

  // Single instance pointer for the static wrappers
  static MotorTask* instance_;

  //Motor Hardware pointer
  DRV883* motor_ = nullptr;

  // Duration constants (ms) used to schedule transitions
  // Use these when scheduling run/wait intervals.
  static constexpr uint32_t WAIT_MS = 1000;
  static constexpr uint32_t RUN_MS  = 1000;

  // Runtime scheduled timestamps (millis) for transitions. These store the
  // absolute time (millis()) at which the current wait/run period expires.
  // Initialized to 0 meaning "not scheduled".
  uint32_t wait_until_ms_ = 0;
  uint32_t run_until_ms_  = 0;

  // FSM + states
  State state_wait_{WAIT, "WAIT", &MotorTask::exec_wait};
  State state_run_ {RUN,  "RUN",  &MotorTask::exec_run };
  State* states_[2] = { &state_wait_, &state_run_ };
  FSM fsm_;
};
