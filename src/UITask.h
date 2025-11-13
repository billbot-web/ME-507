#pragma once
#include <Arduino.h>
#include "FSM.h"
#include "State.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "taskqueue.h"
#include "taskshare.h"  // Share<T>

/**
 * @brief Simple serial UI task with three states: WAIT_FOR_INPUT, VELOCITY_RUN, POSITION_RUN.
 *
 * Commands on Serial:
 *  - 'v' : velocity display/run
 *  - 'p' : position display/run
 *  - 's' : stop (return to wait)
 *  - 'z' : zero encoder (forwarded as command)
 *  - 'h' or '?' : help
 */
class UITask {
public:
  /// Commands that can be casted to queue
    enum class Command {     
        STOP = 0,       ///< Stop encoder readings
        RUN = 1,         ///< Start encoder readings
        ZERO = 2        ///< Reset position to zero
    };

  enum StateId : uint8_t {
    WAIT_FOR_INPUT = 0,
    VELOCITY_RUN   = 1,
  };

  /**
   * @brief Construct UI task.
   * @param positionShare Pointer to position share (may be nullptr)
   * @param velocityShare Pointer to velocity share (may be nullptr)
   * @param encoderCmdQueue Queue handle to send commands to EncoderTask (may be nullptr)
   * @param updateMs UI update/scan period in ms
   */
  UITask(Share<float>* positionShare,
    Share<float>*   velocityShare,
    Share<int8_t>*  vref,
    Share<int8_t>*  cmdShare,
    uint32_t        updateMs = 200) noexcept;

  void start(uint8_t priority = 1, int8_t core = 1);
  void update() noexcept { fsm_.run_curstate(); }
  // FreeRTOS entry function is defined in the .cpp as a C-linkage function
  // extern "C" void ui_task_func(void* pvParameters);

  // Public accessors used by the C-style task entry
  uint32_t get_updateMs() const noexcept { return updateMs_; }
  static void set_instance(UITask* inst) { instance_ = inst; }

private:
  // Shares / queue
  Share<float>* positionShare_ = nullptr;
  Share<float>*   velocityShare_ = nullptr;
  Share<int8_t>*  vref_ = nullptr;
  Share<int8_t>*  cmdShare_ = nullptr;

  // Timing
  uint32_t updateMs_ = 200;
  uint32_t lastMenuPrintMs_ = 0;
  uint32_t lastValuePrintMs_ = 0;
  // Whether we've printed the menu since entering WAIT state
  bool menuPrinted_ = false;
  // Input buffer for numeric entry from Serial
  char inputBuf_[32];
  size_t inputPos_ = 0;

  // FSM
    // FSM + states
  State state_wait_{0, "WAIT_FOR_INPUT", &UITask::exec_waitForInput};
  State state_velocity_run_{1, "VELOCITY_RUN", &UITask::exec_velocityRun};
  State* states_[2] = { &state_wait_, &state_velocity_run_ };
  FSM fsm_;

  // State functions
  static uint8_t exec_waitForInput();
  static uint8_t exec_velocityRun();

  // Helpers
  static void printHelpOnce();
  static void sendEncoderCmdZero();
  static void sendEncoderCmdStart();
  static void sendEncoderCmdStop();

  // Singleton for static callbacks
  static UITask* instance_;
};

