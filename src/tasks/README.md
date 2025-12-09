# FreeRTOS Task Controllers

High-level task implementations coordinating system behavior, sensor fusion, and user interaction using FreeRTOS multitasking.

## 📂 Files

- **MotorTask.cpp/h** - Multi-mode motor control with FSM-based state management
- **EncoderTask.cpp/h** - High-frequency position/velocity measurement and feedback
- **CameraTask.cpp/h** - Computer vision processing for LED detection and tracking
- **ControllerTask.cpp/h** - System coordinator and global mode management
- **UITask.cpp/h** - Web server, WebSocket telemetry, and command processing
- **IMU_TASK.cpp/h** - Inertial measurement and orientation sensing

## 🎯 Task Architecture

All tasks inherit from FreeRTOS task base and follow these patterns:
- **Fixed-period execution** using `vTaskDelayUntil()`
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

## 🚗 MotorTask

### Overview
FSM-based motor controller supporting multiple operating modes with PID control, encoder feedback, and camera-based tracking.

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
- **PID Control**: Separate tuning for position and camera modes
- **Saturation Handling**: Anti-windup with integral clamping
- **Smooth Transitions**: Velocity ramping and position interpolation
- **Safety Limits**: Software position bounds and velocity caps

### API
```cpp
class MotorTask : public Task {
public:
    MotorTask(const char* name, DRV883* motor, Encoder* encoder,
              QueueHandle_t cmd_queue);
    void run(void* params) override;
    void set_mode(MotorMode mode);
    void set_target_velocity(int16_t vel);
    void set_target_position(int32_t pos);
    void set_pid_gains(float kp, float ki, float kd);
    int32_t get_position();
    float get_velocity();
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

// Velocity limits
#define MAX_VELOCITY 200  // counts/sec
#define MAX_PWM 255       // 0-255 range

// Position limits
#define MIN_POSITION -5000
#define MAX_POSITION 5000
```

### Usage Example
```cpp
// Create task
MotorTask panMotor("PanMotor", &motor_drv, &encoder, cmd_queue);
panMotor.begin();

// Position control
panMotor.set_mode(POSITION_MODE);
panMotor.set_target_position(1000);

// Velocity control
panMotor.set_mode(VELOCITY_MODE);
panMotor.set_target_velocity(50);

// Camera tracking
panMotor.set_mode(CAMERA_MODE);
// Error signal sent via queue from CameraTask
```

## 📏 EncoderTask

### Overview
High-frequency task that continuously reads encoder positions, calculates velocities, and provides feedback to motor controllers.

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

### API
```cpp
class EncoderTask : public Task {
public:
    EncoderTask(const char* name, Encoder* encoder);
    void run(void* params) override;
    int32_t get_position();      // Thread-safe position read
    float get_velocity();         // Thread-safe velocity read
    void reset_position();        // Zero the encoder
};
```

### Velocity Calculation
```cpp
// Simple differentiation
velocity = (position_now - position_prev) / delta_time;

// Exponential smoothing (reduces noise)
velocity_filtered = alpha * velocity_raw + (1 - alpha) * velocity_prev;
```

### Usage Example
```cpp
EncoderTask panEncoder("PanEncoder", &encoder_pan);
panEncoder.begin();

// Read from motor control loop
int32_t pos = panEncoder.get_position();
float vel = panEncoder.get_velocity();

if (abs(pos) > MAX_POSITION) {
    // Position limit exceeded!
    motor.brake();
}
```

## 📸 CameraTask

### Overview
Computer vision task that captures frames, detects LED targets using HSV thresholding, and computes visual servoing error signals for tracking.

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

### API
```cpp
class CameraTask : public Task {
public:
    CameraTask(const char* name, OV5640_camera* cam,
               QueueHandle_t error_queue);
    void run(void* params) override;
    void set_hsv_threshold(int h_min, int h_max, int s_min, int v_min);
    bool is_target_detected();
    void get_target_position(int* x, int* y);
};
```

### Error Signal Format
```cpp
struct TrackingError {
    int16_t x_error;   // Pixels from center (-320 to +320)
    int16_t y_error;   // Pixels from center (-240 to +240)
    bool valid;        // Target detected flag
    uint32_t timestamp;
};
```

### HSV Tuning
```cpp
// Red LED example
cameraTask.set_hsv_threshold(
    160,  // Hue min (red = 0/180 in HSV)
    180,  // Hue max
    100,  // Saturation min (vibrant colors)
    100   // Value min (brightness)
);

// Green LED
cameraTask.set_hsv_threshold(60, 80, 100, 100);

// Blue LED
cameraTask.set_hsv_threshold(100, 120, 100, 100);
```

## 🎮 ControllerTask

### Overview
High-level system coordinator that manages global operating modes, coordinates task interactions, and handles mode transitions.

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

### Mode Transition Logic
```cpp
// Mode change validation
bool can_transition(Mode from, Mode to) {
    if (from == CALIBRATE && !is_calibrated) {
        return false;  // Must complete calibration
    }
    if (to == TRACKER && !camera.is_initialized()) {
        return false;  // Camera required
    }
    return true;
}
```

### API
```cpp
class ControllerTask : public Task {
public:
    ControllerTask(const char* name);
    void run(void* params) override;
    void set_mode(SystemMode mode);
    SystemMode get_mode();
    bool is_calibrated();
};
```

### Usage Example
```cpp
ControllerTask controller("Controller");
controller.begin();

// User clicks "Tracker" button in UI
if (controller.get_mode() != TRACKER) {
    controller.set_mode(TRACKER);
    
    // Configure motors for camera mode
    panMotor.set_mode(CAMERA_MODE);
    tiltMotor.set_mode(CAMERA_MODE);
}
```

## 🌐 UITask

### Overview
Web server task providing WiFi connectivity, HTTP file serving, WebSocket telemetry streaming, and remote command processing.

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

### API
```cpp
class UITask : public Task {
public:
    UITask(const char* name);
    void run(void* params) override;
    void send_telemetry(TelemetryData data);
    void set_wifi_credentials(const char* ssid, const char* pass);
    bool is_connected();
};
```

### Telemetry Data Structure
```cpp
struct TelemetryData {
    uint32_t timestamp;
    int32_t pan_position;
    int32_t tilt_position;
    float pan_velocity;
    float tilt_velocity;
    float imu_roll;
    float imu_pitch;
    float imu_yaw;
    int16_t tracking_error_x;
    int16_t tracking_error_y;
    SystemMode current_mode;
};
```

### WebSocket Protocol
```cpp
// Incoming command
void onWebSocketMessage(String message) {
    JsonDocument doc;
    deserializeJson(doc, message);
    
    String type = doc["type"];
    if (type == "set_mode") {
        controller.set_mode(doc["value"]);
    } else if (type == "joystick") {
        panMotor.set_target_velocity(doc["pan_velocity"]);
        tiltMotor.set_target_velocity(doc["tilt_velocity"]);
    }
}

// Outgoing telemetry
void sendTelemetry() {
    JsonDocument doc;
    doc["type"] = "telemetry";
    doc["pan_position"] = encoder_pan.get_position();
    doc["tilt_position"] = encoder_tilt.get_position();
    // ... add more fields
    
    String json;
    serializeJson(doc, json);
    ws.textAll(json);
}
```

## 🧭 IMU_TASK

### Overview
Inertial measurement task that reads BNO055 sensor data, provides orientation information, and detects motion events.

### Responsibilities
- Read accelerometer, gyroscope, magnetometer data
- Get fused orientation (Euler angles or quaternions)
- Monitor sensor calibration status
- Detect motion/vibration for stability assessment

### Key Features
- **100 Hz update rate** (10ms period)
- **Automatic calibration**: Monitor and report cal status
- **Temperature compensation**: Built-in BNO055 feature
- **Coordinate transforms**: Convert between sensor and world frames

### API
```cpp
class IMU_TASK : public Task {
public:
    IMU_TASK(const char* name, Adafruit_BNO055* imu);
    void run(void* params) override;
    void get_euler(float* roll, float* pitch, float* yaw);
    void get_accel(float* x, float* y, float* z);
    uint8_t get_calibration_status();
    bool is_calibrated();
};
```

### Usage Example
```cpp
IMU_TASK imuTask("IMU", &bno055);
imuTask.begin();

// Read orientation
float roll, pitch, yaw;
imuTask.get_euler(&roll, &pitch, &yaw);

// Check if sensor is calibrated
if (imuTask.is_calibrated()) {
    // Use IMU data for stability compensation
    float tilt_compensation = sin(pitch * DEG_TO_RAD);
    motor_tilt.add_feedforward(tilt_compensation);
}
```

## 📊 Task Timing Analysis

| Task | Period | Execution Time | Deadline | Stack Size |
|------|--------|----------------|----------|------------|
| EncoderTask | 1ms | ~50μs | 1ms | 2KB |
| MotorTask | 10ms | ~200μs | 10ms | 4KB |
| CameraTask | 100ms | ~80ms | 100ms | 8KB |
| ControllerTask | 50ms | ~100μs | 50ms | 3KB |
| IMU_TASK | 10ms | ~500μs | 10ms | 3KB |
| UITask | 100ms | ~5ms | 100ms | 8KB |

## 🔄 Inter-Task Communication

### Queue-Based Messaging
```cpp
// Create queues in main.cpp
QueueHandle_t motor_cmd_queue = xQueueCreate(10, sizeof(MotorCommand));
QueueHandle_t tracking_error_queue = xQueueCreate(5, sizeof(TrackingError));

// Producer (CameraTask)
TrackingError error;
error.x_error = led_x - FRAME_CENTER_X;
error.y_error = led_y - FRAME_CENTER_Y;
xQueueSend(tracking_error_queue, &error, 0);

// Consumer (MotorTask)
TrackingError error;
if (xQueueReceive(tracking_error_queue, &error, 0) == pdTRUE) {
    set_target_position(current_pos + error.x_error);
}
```

### Shared Memory with Mutex
```cpp
// Global shared data
SemaphoreHandle_t encoder_mutex;
volatile int32_t shared_encoder_position;

// Writer (EncoderTask)
xSemaphoreTake(encoder_mutex, portMAX_DELAY);
shared_encoder_position = encoder.get_position();
xSemaphoreGive(encoder_mutex);

// Reader (MotorTask)
xSemaphoreTake(encoder_mutex, portMAX_DELAY);
int32_t pos = shared_encoder_position;
xSemaphoreGive(encoder_mutex);
```

## 🐛 Debugging Tasks

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

## 📚 Related Documentation

- [Hardware Drivers](../hardware/README.md) - Low-level peripheral interfaces
- [FSM Framework](../fsm/README.md) - State machine design pattern
- [Software Architecture](../README.md) - Overall system design
- [Web Interface](../../data/README.md) - UI task implementation details

## 🚀 Best Practices

1. **Always use `vTaskDelay()` or `vTaskDelayUntil()`** - Never use `delay()` in tasks
2. **Check queue return values** - Handle full/empty conditions gracefully
3. **Use appropriate priorities** - Time-critical tasks get higher priority
4. **Size stacks conservatively** - Monitor high water mark and adjust
5. **Protect shared resources** - Use mutexes for all shared data
6. **Avoid blocking in high-priority tasks** - Defer heavy work to lower priority
7. **Handle task errors** - Don't let exceptions crash the RTOS
8. **Document inter-task dependencies** - Make communication patterns clear
