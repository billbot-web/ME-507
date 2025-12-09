#pragma once
#include <cstdint>

/**
 * @file State.h
 * @brief Table-driven state descriptor with execution behavior.
 *
 * A State is defined by:
 *  - a small integer ID
 *  - a const char* name (for debugging)
 *  - an execute function that implements the state's behavior and may
 *    return the next state ID to transition to
 *
 * The FSM is execute-driven: it calls the current state's execute() and
 * honors any returned next-state ID to perform forced transitions.
 */
class State {
public:
  /// Execute signature: function that implements state behavior and
  /// returns the next state's ID (or negative to indicate no forced change).
  using Execute = uint8_t (*)();

  /// Construct a state with id, name, and behavior.
  constexpr State(uint8_t id, const char* name, Execute exec) noexcept
    : id_(id), name_(name), exec_(exec) {}

  /// @return small integer ID for external reference
  constexpr uint8_t get_id() const noexcept { return id_; }

  /// @return state name for diagnostics
  constexpr const char* get_name() const noexcept { return name_; }

  /// Execute this state's behavior and return the next state's ID.
  /// If the execute function is null, returns -1 (no forced transition).
  inline uint8_t execute() const noexcept { return exec_ ? exec_() : -1; }

private:
  uint8_t         id_;
  const char* name_;
  Execute     exec_;
};
