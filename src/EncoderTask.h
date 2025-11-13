#pragma once
#include <Arduino.h>
#include "Encoder.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "taskqueue.h"
#include "taskshare.h"  // For Share<T>

/**
 * @brief Task wrapper for encoder reading with simple WAIT/RUN FSM states.
 */
class EncoderTask {
public:
    /// State IDs (index into state table)
    enum StateId {
        WAIT = 0,   ///< Waiting for command
        RUN = 1     ///< Reading encoder
    };

  
    /**
     * @brief Construct encoder task with shares and queue.
     * @param encoder Pointer to Encoder object
     * @param positionShare Share for current position
     * @param velocityShare Share for current velocity
     * @param cmdQueue Queue for commands
     * @param updateMs Update period in milliseconds
     */
    EncoderTask(Encoder* encoder,
                Share<float>* positionShare,
                Share<float>* velocityShare,
                Share<int8_t>* cmdShare,
                uint32_t updateMs = 10) noexcept;

    void update() noexcept { fsm_.run_curstate(); }
    
private:
    // Hardware / timing config
    Encoder* encoder_;
    uint32_t updateMs_;

    // Shared resources
    Share<float>* positionShare_;
    Share<float>* velocityShare_;
    Share<int8_t>* cmdShare_;

    // State management
    static State stateTable_[];

    // FSM + states
    State state_wait_{WAIT, "WAIT", &EncoderTask::exec_wait};
    State state_run_ {RUN,  "RUN",  &EncoderTask::exec_run };
    State* states_[2] = { &state_wait_, &state_run_ };
    FSM fsm_;
    // State functions
    // Static wrappers used for State C-style function pointers
    static uint8_t exec_wait() noexcept;
    static uint8_t exec_run() noexcept;

    /// Singleton instance pointer for static callbacks
    static EncoderTask* instance_;
};