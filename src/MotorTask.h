/**
 * @file MotorTask.h
 * @brief FreeRTOS task for comprehensive motor control with PID velocity and position control
 * 
 * This file implements a sophisticated motor control task that provides both velocity
 * and position control modes using PID controllers. The task operates within a
 * finite state machine framework, allowing for clean state transitions and
 * command-based operation from other system components.
 * 
 * Key Features:
 * - Dual-mode operation: velocity control and position control
 * - PID controllers for precise motion control
 * - FSM-based state management (WAIT/VELOCITY_RUN/POSITION_RUN)
 * - Thread-safe communication via Share<T> objects
 * - DRV883 motor driver integration
 * - Real-time encoder feedback processing
 * 
 * @author Motor Control Team
 * @date November 2025
 * @version 2.3
 * 
 * @see MotorTask.cpp Implementation details and PID tuning parameters
 * @see DRV883.h Motor driver hardware interface
 */

#pragma once
#include "FSM.h"
#include "DRV883.h"
#include "taskqueue.h"
#include "taskshare.h"  // For Share<T>
#include "PID.h"        // For PID controllers

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
  enum Ids { WAIT = 0, VRUN = 1 , PRUN = 2};

  // Single instance pointer for the static wrappers
  static MotorTask* instance_;

  // Accept a pointer to the motor driver (caller retains ownership).
  // Signature: motor, command queue, optional velocity share, and signed vref share
  MotorTask(DRV883* motor, Share<uint8_t>* cmdShare, Share<float>* veloShare, Share<int8_t>* vref, 
    Share<float>* posShare, Share<int16_t>* posref) noexcept;

  /// Run one FSM tick: scan states and execute the selected state's behavior
  void update() noexcept { fsm_.run_curstate(); }

private:
  // Static wrappers used for State C-style function pointers
  static uint8_t exec_wait() noexcept;
  static uint8_t exec_velorun() noexcept;
  static uint8_t exec_posrun() noexcept;


  //Motor Hardware pointer
  DRV883* motor_ = nullptr;
  // Command queue pointer
  // Command share pointer (values: 0=STOP,1=RUN,2=ZERO)
  Share<uint8_t>* cmdShare_ = nullptr;
  // Velocity share pointer (desired velocity / effort)
  Share<float>* veloShare_ = nullptr;
  // Signed reference share pointer (-100..100)
  Share<int8_t>* vref_ = nullptr;
  // Position share pointer (current position)
  Share<float>* posShare_ = nullptr;
  // Position reference share pointer
  Share<int16_t>* posref_ = nullptr;
  // Current state (0=wait, 1=run)
  int state_ = 0;

  // Last applied motor effort (-100..100). Preserved when vref share is absent.
  int last_effort_ = 0;

  // Runtime scheduled timestamps (millis) for transitions. These store the
  // absolute time (millis()) at which the current wait/run period expires.
  // Initialized to 0 meaning "not scheduled".
  uint32_t wait_until_ms_ = 0;
  uint32_t run_until_ms_  = 0;

  // PID Controllers
  PID velocity_pid_;     // PID controller for velocity control
  PID position_pid_;     // PID controller for position control

  // FSM + states
  State state_wait_{WAIT, "WAIT", &MotorTask::exec_wait};
  State state_velorun_ {VRUN,  "RUN",  &MotorTask::exec_velorun };
  State state_posrun_ {PRUN,  "RUN",  &MotorTask::exec_posrun };
  State* states_[3] = { &state_wait_, &state_velorun_, &state_posrun_ };
  FSM fsm_;
};
