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
#include "../fsm/FSM.h"
#include "../hardware/DRV883.h"
#include "taskqueue.h"
#include "taskshare.h"  // For Share<T>
#include "PID.h"        // For PID controllers

/**
 * @class MotorTask
 * @brief Multi-mode motor control task with velocity, position, and camera-based servoing
 *
 * This class encapsulates sophisticated motor control logic using a finite state machine
 * architecture. It supports four distinct operating modes with seamless transitions
 * between them based on command inputs.
 * 
 * Operating Modes:
 * - **WAIT**: Motors stopped, awaiting command
 * - **VRUN**: Open-loop velocity control with stiction compensation
 * - **PRUN**: Closed-loop position control using encoder feedback and PID
 * - **CAMERA_PRUN**: Visual servoing using camera pixel error as control signal
 * 
 * Architecture Details:
 * - Uses static wrapper pattern to bridge C-style FSM callbacks to instance methods
 * - PID controllers are externally managed (pointers) for independent gain tuning
 * - Thread-safe communication via Share<T> objects for inter-task coordination
 * - Position safety limits prevent mechanical damage (tilt motor only)
 * - Stiction compensation improves low-speed tracking performance
 * 
 * @note State expects plain C function pointers, so static wrappers and instance_ pointer
 *       provide the bridge to access member data and methods
 * @see MotorTask.cpp for PID tuning values and stiction compensation parameters
 * @see DRV883.h for motor driver hardware interface
 */
class MotorTask {
public:
  /**
   * @enum Ids
   * @brief FSM state identifiers for motor control modes
   */
  enum Ids { 
    WAIT = 0,         ///< Motors stopped, idle state
    VRUN = 1,         ///< Velocity control mode
    PRUN = 2,         ///< Position control mode
    CAMERA_PRUN = 3   ///< Camera-based position control mode
  };

  /// @brief Single instance pointer for static wrapper functions to access member data
  static MotorTask* instance_;

  /**
   * @brief Construct a new Motor Task object with full control capabilities
   * 
   * @param motor Pointer to DRV883 motor driver (caller retains ownership)
   * @param cmdShare Command share for mode selection (0=WAIT, 1=VRUN, 2=PRUN, 3=CAMERA_PRUN)
   * @param veloShare Velocity setpoint share for VRUN mode (degrees/sec)
   * @param vref Velocity reference share for open-loop effort (-100 to 100)
   * @param posShare Current position share from encoder (degrees)
   * @param posref Position setpoint share for PRUN mode (degrees)
   * @param velocity_pid Pointer to externally managed velocity PID controller
   * @param position_pid Pointer to externally managed position PID controller
   * @param cameraErr Optional camera pixel error share for CAMERA_PRUN mode (pixels)
   * @param isTiltMotor True if this is tilt axis (enables position limits), false for pan
   */
  MotorTask(DRV883* motor, Share<uint8_t>* cmdShare, Share<float>* veloShare, Share<int8_t>* vref, 
    Share<float>* posShare, Share<int16_t>* posref, PID* velocity_pid, PID* position_pid, 
    Share<int16_t>* cameraErr = nullptr, bool isTiltMotor = false) noexcept;

  /**
   * @brief Execute one FSM tick - run current state's behavior and check for transitions
   * @note Should be called periodically from FreeRTOS task function
   */
  void update() noexcept { fsm_.run_curstate(); }

  /**
   * @brief Set the static instance pointer for wrapper function access
   * @param inst Pointer to MotorTask instance
   * @note Must be called before each update() to ensure correct instance context
   */
  static void set_instance(MotorTask* inst) { instance_ = inst; }

private:
  // -------------------------------------------------------------------------
  // Static Wrapper Functions (FSM State Executors)
  // -------------------------------------------------------------------------
  
  /**
   * @brief WAIT state executor - motors stopped, awaiting command
   * @return Next state ID to transition to
   */
  static uint8_t exec_wait() noexcept;
  
  /**
   * @brief VRUN state executor - velocity control mode
   * @return Next state ID to transition to
   */
  static uint8_t exec_velorun() noexcept;
  
  /**
   * @brief PRUN state executor - position control mode
   * @return Next state ID to transition to
   */
  static uint8_t exec_posrun() noexcept;
  
  /**
   * @brief CAMERA_PRUN state executor - camera-based position control
   * @return Next state ID to transition to
   */
  static uint8_t exec_camera_posrun() noexcept;

  // -------------------------------------------------------------------------
  // Hardware and Communication Interfaces
  // -------------------------------------------------------------------------

  DRV883* motor_ = nullptr;                  ///< Motor driver hardware interface
  Share<uint8_t>* cmdShare_ = nullptr;       ///< Command share (0=WAIT,1=VRUN,2=PRUN,3=CAMERA_PRUN)
  Share<float>* veloShare_ = nullptr;        ///< Velocity setpoint share (degrees/sec)
  Share<int8_t>* vref_ = nullptr;            ///< Velocity reference share (-100 to 100)
  Share<float>* posShare_ = nullptr;         ///< Current position share (degrees)
  Share<int16_t>* posref_ = nullptr;         ///< Position setpoint share (degrees)
  Share<int16_t>* cameraErr_ = nullptr;      ///< Camera pixel error share (optional)
  
  // -------------------------------------------------------------------------
  // Configuration and State
  // -------------------------------------------------------------------------
  
  bool isTiltMotor_ = false;                 ///< True if tilt motor (enables position limits)
  int state_ = 0;                            ///< Current FSM state ID
  int last_effort_ = 0;                      ///< Last applied motor effort (-100 to 100)
  uint32_t wait_until_ms_ = 0;               ///< Scheduled wait period end time (millis)
  uint32_t run_until_ms_  = 0;               ///< Scheduled run period end time (millis)

  // -------------------------------------------------------------------------
  // PID Controllers (Externally Managed)
  // -------------------------------------------------------------------------
  
  PID* velocity_pid_;     ///< Velocity PID controller pointer (tuned in main.cpp)
  PID* position_pid_;     ///< Position PID controller pointer (tuned in main.cpp)

  // -------------------------------------------------------------------------
  // FSM State Objects
  // -------------------------------------------------------------------------
  
  State state_wait_{WAIT, "WAIT", &MotorTask::exec_wait};                          ///< WAIT state object
  State state_velorun_ {VRUN,  "VRUN",  &MotorTask::exec_velorun };                ///< VRUN state object
  State state_posrun_ {PRUN,  "PRUN",  &MotorTask::exec_posrun };                  ///< PRUN state object
  State state_camera_posrun_ {CAMERA_PRUN, "CAMERA_PRUN", &MotorTask::exec_camera_posrun}; ///< CAMERA_PRUN state object
  State* states_[4] = { &state_wait_, &state_velorun_, &state_posrun_, &state_camera_posrun_ }; ///< State array for FSM
  FSM fsm_;  ///< Finite state machine manager
};
