# Finite State Machine Framework

Lightweight, reusable FSM implementation for modeling complex behavioral control in embedded systems. Used throughout SportTrackr for task state management.

## 📂 Files

- **FSM.cpp/h** - State machine engine implementation
- **State.h** - State base class definition
- **FSM.md** - Detailed design documentation

## 🎯 Overview

The FSM framework provides a simple, table-driven approach to finite state machines with minimal memory overhead and execution cost. States are defined as function pointers that return the next state ID, enabling flexible and maintainable behavioral control.

### Design Philosophy
- **Execute-Driven**: States are functions that execute and return next state
- **Minimal Overhead**: No virtual functions, no dynamic allocation required
- **Embedded-Friendly**: Suitable for resource-constrained microcontrollers
- **Type-Safe**: Compile-time state table validation

## 🧩 Core Components

### State Class

```cpp
class State {
public:
    State(int id, const char* name, Execute exec);
    
    int execute();           // Run state behavior, return next state ID
    int get_id();           // Get this state's unique ID
    const char* get_name(); // Get state name (for debugging)
    
private:
    int _id;
    const char* _name;
    Execute _exec;  // typedef int (*Execute)();
};
```

**State Function Signature:**
```cpp
int state_function() {
    // State behavior logic
    return NEXT_STATE_ID;  // or current ID to stay
}
```

### FSM Class

```cpp
class FSM {
public:
    FSM(State** states, std::size_t count);
    
    void run_curstate();      // Execute current state once
    State* get_curstate();    // Get current state object
    std::size_t get_curstateidx();  // Get current state index
    std::size_t get_count();  // Get total number of states
    
private:
    State** _states;
    std::size_t _count;
    std::size_t _curstateidx;
};
```

## 🚀 Usage Example

### Basic State Machine

```cpp
// Define state IDs
enum StateID {
    STATE_IDLE = 0,
    STATE_RUNNING,
    STATE_ERROR,
    STATE_COUNT
};

// Define state behaviors
int state_idle() {
    Serial.println("Idle state");
    
    if (start_button_pressed()) {
        return STATE_RUNNING;
    }
    return STATE_IDLE;  // Stay in idle
}

int state_running() {
    Serial.println("Running state");
    
    if (error_detected()) {
        return STATE_ERROR;
    }
    if (stop_button_pressed()) {
        return STATE_IDLE;
    }
    
    // Do work...
    
    return STATE_RUNNING;  // Continue running
}

int state_error() {
    Serial.println("Error state");
    
    if (error_cleared()) {
        return STATE_IDLE;
    }
    return STATE_ERROR;
}

// Create state objects
State idle(STATE_IDLE, "IDLE", state_idle);
State running(STATE_RUNNING, "RUNNING", state_running);
State error(STATE_ERROR, "ERROR", state_error);

// Build state table
State* state_table[] = {&idle, &running, &error};

// Create FSM (starts in first state)
FSM fsm(state_table, STATE_COUNT);

// Run FSM in loop
void loop() {
    fsm.run_curstate();
    delay(100);
}
```

## 🎮 MotorTask FSM Example

### State Diagram
```
┌──────────┐
│  IDLE    │◄──────────────────┐
└────┬─────┘                   │
     │ start_velocity          │
     ▼                         │ stop
┌──────────┐                   │
│ VELOCITY │                   │
│  MODE    │───────────────────┤
└────┬─────┘  position_cmd     │
     │                         │
     ▼                         │
┌──────────┐                   │
│ POSITION │                   │
│  MODE    │───────────────────┤
└────┬─────┘  camera_active    │
     │                         │
     ▼                         │
┌──────────┐                   │
│ CAMERA   │                   │
│  MODE    │───────────────────┘
└──────────┘
```

### Implementation

```cpp
enum MotorState {
    MOTOR_IDLE = 0,
    MOTOR_VELOCITY,
    MOTOR_POSITION,
    MOTOR_CAMERA,
    MOTOR_STATE_COUNT
};

// Global motor control variables
extern DRV883 motor;
extern Encoder encoder;
extern int16_t target_velocity;
extern int32_t target_position;

// State: IDLE
int motor_idle() {
    motor.brake();
    
    // Wait for command
    if (command_queue_has_data()) {
        MotorCommand cmd = read_command();
        
        if (cmd.type == CMD_VELOCITY) {
            target_velocity = cmd.value;
            return MOTOR_VELOCITY;
        } else if (cmd.type == CMD_POSITION) {
            target_position = cmd.value;
            return MOTOR_POSITION;
        }
    }
    
    return MOTOR_IDLE;
}

// State: VELOCITY MODE
int motor_velocity() {
    // Open-loop velocity control
    motor.set_speed(target_velocity);
    
    // Check for mode change
    if (stop_command_received()) {
        return MOTOR_IDLE;
    }
    if (position_command_received()) {
        target_position = read_position_command();
        return MOTOR_POSITION;
    }
    
    return MOTOR_VELOCITY;
}

// State: POSITION MODE
int motor_position() {
    // PID position control
    int32_t current_pos = encoder.get_position();
    int32_t error = target_position - current_pos;
    
    float pid_output = calculate_pid(error);
    motor.set_speed((int16_t)pid_output);
    
    // Check for mode change
    if (stop_command_received()) {
        return MOTOR_IDLE;
    }
    if (camera_active()) {
        return MOTOR_CAMERA;
    }
    
    return MOTOR_POSITION;
}

// State: CAMERA MODE
int motor_camera() {
    // Visual servoing control
    TrackingError error;
    if (get_camera_error(&error)) {
        target_position = encoder.get_position() + error.x_error;
    }
    
    // Use position control with camera-derived target
    int32_t current_pos = encoder.get_position();
    int32_t pos_error = target_position - current_pos;
    
    float pid_output = calculate_pid(pos_error);
    motor.set_speed((int16_t)pid_output);
    
    // Check for mode change
    if (stop_command_received() || !camera_active()) {
        return MOTOR_IDLE;
    }
    
    return MOTOR_CAMERA;
}

// Setup FSM
State motor_idle_state(MOTOR_IDLE, "IDLE", motor_idle);
State motor_velocity_state(MOTOR_VELOCITY, "VELOCITY", motor_velocity);
State motor_position_state(MOTOR_POSITION, "POSITION", motor_position);
State motor_camera_state(MOTOR_CAMERA, "CAMERA", motor_camera);

State* motor_states[] = {
    &motor_idle_state,
    &motor_velocity_state,
    &motor_position_state,
    &motor_camera_state
};

FSM motor_fsm(motor_states, MOTOR_STATE_COUNT);

// Task run loop
void MotorTask::run(void* params) {
    TickType_t last_wake = xTaskGetTickCount();
    
    while (1) {
        motor_fsm.run_curstate();
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(10));  // 100 Hz
    }
}
```

## 🔧 Advanced Features

### State Entry/Exit Actions

```cpp
int last_state = -1;

int state_with_entry_exit() {
    State* current = fsm.get_curstate();
    
    // Entry action (only on first execution)
    if (current->get_id() != last_state) {
        Serial.println("Entering state");
        initialize_resources();
    }
    
    // Main state behavior
    int next_state = do_work();
    
    // Exit action (only if transitioning)
    if (next_state != current->get_id()) {
        Serial.println("Exiting state");
        cleanup_resources();
    }
    
    last_state = current->get_id();
    return next_state;
}
```

### Hierarchical State Machines

```cpp
// Parent FSM manages high-level modes
enum SystemMode {
    MODE_IDLE,
    MODE_TRACKING,
    MODE_MANUAL
};

// Child FSM handles tracking sub-states
enum TrackingState {
    TRACK_SEARCH,
    TRACK_LOCKED,
    TRACK_LOST
};

// System FSM transitions to TRACKING mode
int system_tracking_mode() {
    // Run child FSM
    tracking_fsm.run_curstate();
    
    // Parent can override child
    if (stop_button_pressed()) {
        return MODE_IDLE;  // Exit tracking entirely
    }
    
    return MODE_TRACKING;
}
```

### Timed Transitions

```cpp
uint32_t state_entry_time = 0;

int state_with_timeout() {
    if (state_entry_time == 0) {
        state_entry_time = millis();  // First execution
    }
    
    // Timeout after 5 seconds
    if (millis() - state_entry_time > 5000) {
        state_entry_time = 0;
        return NEXT_STATE;
    }
    
    // Normal behavior
    return CURRENT_STATE;
}
```

### Conditional Transitions

```cpp
int state_with_conditions() {
    // Priority-based transitions
    if (critical_error()) {
        return ERROR_STATE;  // Highest priority
    }
    if (warning_detected()) {
        return WARNING_STATE;
    }
    if (task_complete()) {
        return IDLE_STATE;
    }
    
    // Continue working
    return CURRENT_STATE;
}
```

## 📊 Debugging FSMs

### State Transition Logging

```cpp
void FSM::run_curstate() {
    State* prev_state = get_curstate();
    int prev_id = prev_state->get_id();
    
    int next_id = _states[_curstateidx]->execute();
    
    // Validate next state
    if (next_id < 0 || next_id >= _count) {
        Serial.printf("Invalid state ID: %d\n", next_id);
        return;  // Stay in current state
    }
    
    // Log transitions
    if (next_id != prev_id) {
        Serial.printf("Transition: %s -> %s\n",
                      prev_state->get_name(),
                      _states[next_id]->get_name());
    }
    
    _curstateidx = next_id;
}
```

### State Execution Time

```cpp
void FSM::run_curstate() {
    uint32_t start = micros();
    
    int next_id = _states[_curstateidx]->execute();
    
    uint32_t elapsed = micros() - start;
    if (elapsed > 1000) {  // >1ms warning
        Serial.printf("State %s took %luμs\n",
                      _states[_curstateidx]->get_name(),
                      elapsed);
    }
    
    _curstateidx = next_id;
}
```

### State History

```cpp
#define HISTORY_SIZE 10
int state_history[HISTORY_SIZE];
int history_idx = 0;

void log_state_transition(int state_id) {
    state_history[history_idx] = state_id;
    history_idx = (history_idx + 1) % HISTORY_SIZE;
}

void print_state_history() {
    Serial.println("State History:");
    for (int i = 0; i < HISTORY_SIZE; i++) {
        int idx = (history_idx + i) % HISTORY_SIZE;
        Serial.printf("%d ", state_history[idx]);
    }
    Serial.println();
}
```

## 🎯 Design Patterns

### Moore Machine
Output depends only on current state (no inputs in state function):
```cpp
int state_moore() {
    // Output based on state
    set_output(STATE_OUTPUT_VALUE);
    
    // Transition based on inputs
    if (input_condition()) {
        return NEXT_STATE;
    }
    return CURRENT_STATE;
}
```

### Mealy Machine
Output depends on current state and inputs:
```cpp
int state_mealy() {
    // Output based on state AND input
    if (input_a()) {
        set_output(OUTPUT_A);
    } else if (input_b()) {
        set_output(OUTPUT_B);
    }
    
    // Transition
    return next_state_based_on_input();
}
```

## 📚 Related Documentation

- [Task Controllers](../tasks/README.md) - FSM usage in FreeRTOS tasks
- [Software Architecture](../README.md) - Overall system design
- [FSM.md](FSM.md) - Detailed framework documentation

## ⚡ Performance

- **State execution**: ~1μs overhead (function call + array lookup)
- **Memory**: sizeof(State*) × num_states (typically 16-32 bytes)
- **No dynamic allocation**: All memory statically defined
- **Deterministic**: Constant-time transitions

## 🚀 Best Practices

1. **Keep states small** - Each state function should be <50 lines
2. **Avoid blocking** - No delays or long loops in state functions
3. **Use enums for IDs** - Better readability than magic numbers
4. **Name states clearly** - Descriptive names aid debugging
5. **Document transitions** - State diagrams or comments
6. **Validate inputs** - Check for invalid state IDs
7. **Log transitions** - Track state changes for debugging
8. **Test edge cases** - Verify all transition paths

## 🐛 Common Pitfalls

❌ **Forgetting to return state ID**
```cpp
int bad_state() {
    do_work();
    // Missing return!
}
```

❌ **Returning invalid state ID**
```cpp
int bad_state() {
    return 999;  // Out of bounds!
}
```

❌ **Blocking in state function**
```cpp
int bad_state() {
    delay(1000);  // Blocks entire FSM!
    return CURRENT_STATE;
}
```

✅ **Correct pattern**
```cpp
int good_state() {
    do_work();
    
    if (transition_condition()) {
        return NEXT_STATE;
    }
    return CURRENT_STATE;
}
```
