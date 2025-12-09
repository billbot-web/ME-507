/**
 * @file FSM.h
 * @brief Table-driven Finite State Machine framework with execute-driven state transitions
 * 
 * @author FSM Framework Team
 * @date November 2025
 * @version 2.0
 */
#pragma once
#include <cstddef>
#include "State.h"

/**
 * @file FSM.h
 * @brief Table-driven FSM with automatic state execution.
 *
 * Design:
 *  - The FSM manages state transitions and execution
 *  - On update(), it calls the current state's execute() function
 *  - The execute() function may return a next-state ID to force a transition
 *  - State behavior is encapsulated in the execute() function
 *
 * Priority:
 *  - Higher index = higher priority (because we scan in descending order)
 */
class FSM {
public:
  /// Construct FSM with a provided state table. The FSM does not take ownership.
  FSM(State** states, std::size_t count) noexcept;

  /**
   * @brief Provide the table of states and its length.
   * @param states Pointer to an array of State* (not owned)
   * @param count  Number of entries in the array
   *
   * The FSM does not take ownership; ensure the pointed-to States outlive the FSM.
   */
  // set_states() removed: provide states via constructor instead.

  /**
   * @brief Execute the current state's behavior (initializing to the first entry on startup).
   *
   * Selection rule:
   *   - This FSM is execute-driven (no predicate-based scanning).
   *   - On creation it is initialized with a state table; the first update
   *     will initialize the current state to the first entry in that table.
   *   - Thereafter run_curstate() calls the current state's execute() and
   *     honors any returned next-state ID to perform a forced transition.
   *
   * Note: This function does not return the State*; use `get_curstate()` to
   * query the currently active state when needed.
   */
  void run_curstate() noexcept {
    // Ensure we have a current state; if none, initialize to first state if available
    if (!curState_) {
      if (!states_ || count_ == 0) return;
      curState_ = states_[0];
      curIdx_ = 0;
    }

    // Execute the current state's behavior and obtain a next-state id.
    uint8_t nextId = curState_-> execute();

    // If the execute returned a non-negative value, treat it as the
    // index into the states_ array (IDs == indices). Switch current
    // state to states_[nextId] if it is a valid index and different.
    if (nextId >= 0 && static_cast<std::size_t>(nextId) < count_ && nextId != curIdx_) {
      State* target = states_[static_cast<std::size_t>(nextId)];
      if (target) {
        curState_ = target;
        curIdx_ = static_cast<int>(nextId);
      }
    }
  }

  /// @return Current State* (may be nullptr if none active)
  State* get_curstate() const noexcept { return curState_; }

  void set_curstate(State* new_curState) {curState_ = new_curState; }

  /// @return Current state index in the list; -1 if none active
  int get_curstateidx() const noexcept { return curIdx_; }

  void set_curstateidx(uint8_t stateidx) {curIdx_= stateidx; }

  /// @return Number of states registered
  std::size_t get_count() const noexcept { return count_; }

private:
  State**       states_        = nullptr;
  std::size_t   count_         = 0;
  State*        curState_      = nullptr;
  int           curIdx_        = -1;
};
