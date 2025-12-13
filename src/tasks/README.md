# FreeRTOS Task Controllers

High-level task implementations coordinating system behavior, sensor fusion, and user interaction using FreeRTOS multitasking.

## 📂 Files

- **MotorTask.cpp/h** - Multi-mode motor control with FSM-based state management
- **EncoderTask.cpp/h** - High-frequency position/velocity measurement and feedback
- **CameraTask.cpp/h** - Computer vision processing for LED detection and tracking
- **ControllerTask.cpp/h** - System coordinator and global mode management
- **UITask.cpp/h** - Web server, WebSocket telemetry, and command processing
- **IMU_TASK.cpp/h** - Inertial measurement and orientation sensing

## Task Architecture
All tasks inherit from FreeRTOS task base and follow these patterns:
- **Fixed-period execution** using `vTaskDelay()`
- **Queue-based communication** for inter-task messaging
- **Shared data structures** with mutex protection
- **State machine logic** for complex behaviors

### Task Priority Hierarchy
```
Priority 5 (Highest) - EncoderTask (time-critical feedback)
Priority 4           - MotorTask (control loop)
Priority 3           - CameraTask (vision processing)
Priority 3           - ControllerTask (coordination)
Priority 2           - IMU_TASK (sensor reading)
Priority 1 (Lowest)  - UITask (network/telemetry)
```

## MotorTask

### Overview
FSM-based motor controller supporting multiple operating modes with PID control, encoder feedback, and camera-based tracking.
![MotorTask](https://github.com/user-attachments/assets/bf175469-aede-49b0-97c1-93013fb194d4)


### Operating Modes (States)

1. **IDLE**: Motor disabled, awaiting commands
2. **VELOCITY_MODE**: Open-loop velocity control via PWM
3. **POSITION_MODE**: Closed-loop PID position control
4. **CAMERA_MODE**: Visual servoing using camera error signals
5. **MOTOR_TEST**: Diagnostic mode for testing motor response

### FSM State Transitions
```
        [Command]          [Command]
IDLE ─────────────→ VELOCITY_MODE ←──────────→ POSITION_MODE
  ↑                       ↑                          ↑
  │                       │                          │
  └───────────────────────┴──────────────────────────┘
                    [Stop Command]
                    
                [Camera Active]
        POSITION_MODE ──────────→ CAMERA_MODE
```

### Key Features
- **PID Control**: Separate gains for position and velocity modes
- **Saturation Handling**: Anti-windup with integral clamping
- **Smooth Transitions**: Velocity ramping and position interpolation
- **Stiction Offset**: The motor's high gear ratio leads to a high static friction, especially the tilt motor, which only begins to spin at around 35 effort. To overcome this, we have an offset that is added to the effort
- **Safety Limits**: Software position bounds, only for the tilt motor. The tilt motor can only spin in the range of -10 to 150 degrees

### API
```cpp
class MotorTask: public Task {
public:
    MotorTask(const char* name, DRV883* motor, Shares..., PID* velociypid, PID* positionpid, bool isTiltmotor);
  void update() noexcept { fsm_.run_curstate(); }
  static void set_instance(MotorTask* inst) { instance_ = inst; }

private:
  static uint8_t exec_wait() noexcept;
  static uint8_t exec_velorun() noexcept;
  static uint8_t exec_posrun() noexcept;
  static uint8_t exec_camera_posrun() noexcept;
};
```

### Configuration
```cpp
// PID tuning (in main.cpp)
motorTask->set_pid_gains(
    0.5f,  // Kp - Proportional gain
    0.0f,  // Ki - Integral gain (often zero for position)
    0.1f   // Kd - Derivative gain
);
```

##EncoderTask

### Overview
High-frequency task that continuously reads encoder positions, calculates velocities, and provides feedback to motor controllers.
![EncoderTask](https://github.com/user-attachments/assets/a1128ab2-f0a7-400b-ac40-09c9af808868)

### Responsibilities
- Read hardware PCNT counters (no CPU overhead)
- Calculate velocity via position differentiation
- Detect and handle counter overflows
- Publish data to shared memory with mutex protection

### Key Features
- **1kHz sampling rate** (1ms period)
- **Velocity filtering**: Moving average or exponential smoothing
- **Overflow detection**: 16-bit counter wraparound handling
- **Zero-crossing**: Detect direction changes

## CameraTask

### Overview
Computer vision task that captures frames, detects LED targets using HSV thresholding, and computes visual servoing error signals for tracking.
![CameraTask](https://github.com/user-attachments/assets/64b3e432-3762-4f76-b10b-dca2365b4c6b)

### Processing Pipeline
1. **Frame Capture**: Trigger OV5640 JPEG capture (~60-100ms)
2. **Color Conversion**: RGB → HSV color space
3. **Thresholding**: Isolate LED pixels by hue/saturation/value
4. **Blob Detection**: Find largest contiguous region
5. **Centroid Calculation**: Compute target center (x, y)
6. **Error Computation**: Distance from frame center
7. **Error Publishing**: Send to MotorTask via queue

### Key Features
- **Adaptive Detection**: Auto-adjust thresholds based on ambient light
- **Multi-target Support**: Track multiple LEDs (future)
- **Lost Target Handling**: Timeout and search behavior
- **Frame Buffering**: PSRAM-based double buffering
  
## ControllerTask

### Overview
High-level system coordinator that manages global operating modes, coordinates task interactions, and handles mode transitions.
![ControllerTask](https://github.com/user-attachments/assets/c9e7ed2b-2ec4-4614-8e3a-ce54d2f2d908)

### System Modes

1. **TRACKER**: Autonomous camera-based tracking
   - Camera detects LED
   - Error sent to motors
   - PID control active
   
2. **TELEOP**: Manual teleoperation
   - User commands from web UI
   - Direct velocity control
   - No autonomous behavior

3. **MOTOR_TEST**: Diagnostic mode
   - Open-loop motor testing
   - Manual PWM control
   - Encoder monitoring

4. **CALIBRATE**: Homing and setup
   - Find mechanical limits
   - Zero encoder positions
   - Validate hardware

## UITask

### Overview
Web server task providing WiFi connectivity, HTTP file serving, WebSocket telemetry streaming, and remote command processing.
![UITask](https://github.com/user-attachments/assets/74903a7c-50c6-47e2-aa5f-7b3cec38931c)

### Responsibilities
- **WiFi Management**: Connect to network, handle reconnection
- **HTTP Server**: Serve web UI files (HTML/CSS/JS)
- **WebSocket Server**: Bidirectional real-time communication
- **Telemetry Publishing**: Stream sensor data to clients (10 Hz)
- **Command Processing**: Parse and execute user commands

### Key Features
- **Async Web Server**: Non-blocking HTTP/WebSocket using ESPAsyncWebServer
- **JSON Communication**: Structured data format for telemetry and commands
- **Auto-Reconnect**: WiFi watchdog with exponential backoff
- **CORS Support**: Cross-origin resource sharing for development

## IMUTASK

### Overview
An inertial measurement task that reads BNO055 sensor data, provides orientation information, and detects motion events.
![IMUTask](https://github.com/user-attachments/assets/6972fc89-d5c0-4bb2-843c-6860cba8c45c)

### Responsibilities
- Read accelerometer, gyroscope, magnetometer data
- Get fused orientation (Euler angles or quaternions)
- Monitor sensor calibration status
- Detect motion/vibration for stability assessment

### Key Features
- **5 Hz update rate**
- **Automatic calibration**: Monitor and report cal status
- **Temperature compensation**: Built-in BNO055 feature
- **Coordinate transforms**: Convert between sensor and world frames

##Task Timing Analysis

| Task           | Period | Exec Time | Stack Size |
|----------------|--------|-----------|------------|
| EncoderTask    | 10ms   | ~50μs     | 2KB        |
| MotorTask      | 50ms   | ~200μs    | 2KB        |
| CameraTask     | 100ms  | ~80ms     | 8KB        |
| ControllerTask | 100ms  | ~100μs    | 4KB        |
| IMUTASK        | 200ms  | ~500μs    | 4KB        |
| UITask         | 200ms  | ~5ms      | 8KB        |
## Debugging Tasks

### Stack Overflow Detection
```cpp
void run(void* params) {
    while (1) {
        // Task work...
        
        // Check remaining stack
        UBaseType_t watermark = uxTaskGetStackHighWaterMark(NULL);
        if (watermark < 200) {
            Serial.printf("[%s] Low stack: %d bytes\n", 
                          pcTaskGetName(NULL), watermark);
        }
    }
}
```

### Task State Monitoring
```cpp
// In main loop or debug task
void print_task_stats() {
    char stats[512];
    vTaskList(stats);
    Serial.println("Task Name\tState\tPrio\tStack\tNum");
    Serial.println(stats);
}
```

### Timing Profiling
```cpp
void run(void* params) {
    TickType_t start, end;
    
    while (1) {
        start = xTaskGetTickCount();
        
        // Task execution
        
        end = xTaskGetTickCount();
        uint32_t elapsed = (end - start) * portTICK_PERIOD_MS;
        
        if (elapsed > PERIOD_MS * 0.9) {
            Serial.printf("[%s] Deadline miss: %lums\n", 
                          pcTaskGetName(NULL), elapsed);
        }
    }
}
```

## Related Documentation

- [Hardware Drivers](../hardware/README.md) - Low-level peripheral interfaces
- [FSM Framework](../fsm/README.md) - State machine design pattern
- [Software Architecture](../README.md) - Overall system design
- [Web Interface](../../data/README.md) - UI task implementation details

## Best Practices

1. **Always use `vTaskDelay()` or `vTaskDelayUntil()`** - Never use `delay()` in tasks
2. **Check queue return values** - Handle full/empty conditions gracefully
3. **Use appropriate priorities** - Time-critical tasks get higher priority
4. **Size stacks conservatively** - Monitor high water mark and adjust
5. **Protect shared resources** - Use mutexes for all shared data
6. **Avoid blocking in high-priority tasks** - Defer heavy work to lower priority
7. **Handle task errors** - Don't let exceptions crash the RTOS
8. **Document inter-task dependencies** - Make communication patterns clear
