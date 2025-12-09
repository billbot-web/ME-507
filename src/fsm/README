# Finite State Machine (FSM) Implementation

A lightweight, table-driven FSM implementation designed for embedded systems. This implementation uses execute-driven state transitions with minimal memory overhead.

## Core Components

### State Class
```cpp
class State {
    State(int id, const char* name, Execute exec);
}
```

The `State` class represents individual states in the FSM:

- **Properties**:
  - `id`: Unique identifier for the state
  - `name`: Human-readable state name (for debugging)
  - `exec`: Function pointer to state behavior

- **Key Methods**:
  - `execute()`: Runs the state's behavior
  - `get_id()`: Returns state ID
  - `get_name()`: Returns state name

- **State Execution**:
  - Returns next state ID (it can either return itself or move to a new state)
  - Null exec function defaults to no transition (-1)

### FSM Class
```cpp
class FSM {
    FSM(State** states, std::size_t count);
}
```

The `FSM` class manages state transitions and execution:

- **Construction**:
  - Takes array of State pointers
  - States array is not owned by FSM
  - First state in array is initial state

- **Key Methods**:
  - `run_curstate()`: Execute current state
  - `get_curstate()`: Get active state
  - `get_curstateidx()`: Get active state index
  - `get_count()`: Get total states

## Usage Example

```cpp
// Define state behaviors
int state0_execute() {
    // Do something
    return 0;  // Stay in this state
}

int state1_execute() {
    // Do something else
    return 0;   // Transition to state[0]
}

// Create states
State state0(0, "STATE0", state0_execute);
State state1(1, "STATE1", state1_execute);

// Setup state table
State* states[] = {&state0, &state1};
FSM fsm(states, 2);

// Run FSM
while(1) {
    fsm.run_curstate();
}
```

## Implementation Details

### State Transitions
1. Current state's `execute()` is called
2. If execute returns a new id we change the state
3. Transition occurs if index valid and different from current
4. Negative return values maintain current state

### Memory Management
- FSM does not own state objects
- States must outlive the FSM
- Minimal dynamic allocation (none required)

## Best Practices

1. **State Design**:
   - Keep state behavior functions small
   - Use const member functions where possible
   - Document transition conditions

2. **FSM Usage**:
   - Initialize all states before FSM creation
   - Ensure states outlive the FSM
   - Check state validity in debug builds

3. **Error Handling**:
   - Invalid state IDs are ignored
   - Null state pointers are handled
   - First update initializes to state 0