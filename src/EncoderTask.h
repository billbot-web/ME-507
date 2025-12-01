/**
 * @file EncoderTask.h
 * @brief FreeRTOS task wrapper for encoder management with FSM-based control
 * 
 * @author Encoder Control Team
 * @date November 2025
 * @version 1.5
 */

#pragma once
#include <Arduino.h>
#include "Encoder.h"
#include "freertos/FreeRTOS.h"
#include "State.h"
#include <cstdint>
#include "freertos/task.h"
#include "taskqueue.h"
#include "taskshare.h"  // For Share<T>

/**
 * @brief Task wrapper for encoder reading with simple WAIT/RUN FSM states.
 */
class EncoderTask {
public:
    /// State IDs (index into state table)
     /// Singleton instance pointer for static callbacks
    static EncoderTask* instance_;

    enum StateId {
        WAIT = 0,   ///< Waiting for command
        RUN = 1     ///< Reading encoder
    };

  
    /**
     * @brief Construct encoder task with shares and queue.
     * @param encoder Pointer to Encoder object
     * @param positionShare Share for current position
     * @param velocityShare Share for current velocity
     * @param cmdShare Share for commands
     * @param zeroShare Share for zero command (boolean)
     * @param updateMs Update period in milliseconds
     */
    EncoderTask(Encoder* encoder,
                Share<float>* positionShare,
                Share<float>* velocityShare,
                Share<uint8_t>* cmdShare,
                Share<bool>* zeroShare,
                uint32_t updateMs = 10) noexcept;

    void update() noexcept { fsm_.run_curstate(); }
    
private:
    // Hardware / timing config
    Encoder* encoder_;
    uint32_t updateMs_;

    // Shared resources
    Share<float>* positionShare_;
    Share<float>* velocityShare_;
    Share<uint8_t>* cmdShare_;
    Share<bool>* zeroShare_;

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

    
};