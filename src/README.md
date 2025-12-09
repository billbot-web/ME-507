# Software Architecture

This directory contains all source code for the SportTrackr embedded control system, organized into three main subsystems: hardware drivers, FreeRTOS tasks, and FSM framework.

## 📂 Directory Structure

```
src/
├── hardware/          # Low-level peripheral drivers
│   ├── DRV883.cpp/h           - Motor driver interface
│   ├── Encoder.cpp/h          - Quadrature encoder driver
│   ├── OV5640_camera.cpp/h    - Camera module interface
│   └── Adafruit_BNO055.cpp/h  - IMU sensor driver
├── tasks/             # FreeRTOS task controllers
│   ├── MotorTask.cpp/h        - Multi-mode motor control FSM
│   ├── EncoderTask.cpp/h      - Position/velocity measurement
│   ├── CameraTask.cpp/h       - Computer vision processing
│   ├── ControllerTask.cpp/h   - System coordinator
│   ├── UITask.cpp/h           - Web server & telemetry
│   └── IMU_TASK.cpp/h         - Inertial measurement
├── fsm/               # Finite state machine framework
│   ├── FSM.cpp/h              - State machine engine
│   ├── State.h                - State base class
│   └── FSM.md                 - Design documentation
└── main.cpp           # System initialization & setup
```

## 🏗️ Architecture Overview

### Design Philosophy
The system follows a **modular, task-based architecture** using FreeRTOS for concurrency and finite state machines for complex behavioral control. Each major subsystem runs as an independent task with well-defined responsibilities and communication interfaces.

### Key Principles
- **Separation of Concerns**: Hardware drivers are isolated from application logic
- **State-Driven Behavior**: Complex control modes managed via FSM pattern
- **Thread-Safe Communication**: FreeRTOS queues and shared data structures
- **Real-Time Responsiveness**: Fixed-period tasks with predictable timing
- **Modularity**: Easy to add/remove features without affecting other subsystems

## 🎯 System Components

### 1. Hardware Drivers (`hardware/`)
Low-level peripheral interfaces providing abstraction over ESP32 hardware features.

**[Full Hardware Documentation →](hardware/README.md)**

- **DRV883**: H-bridge motor driver control (PWM, direction, sleep)
- **Encoder**: PCNT-based quadrature decoder for position/velocity
- **OV5640_camera**: Image capture, JPEG compression, LED detection
- **Adafruit_BNO055**: 9-DOF IMU with sensor fusion

### 2. Task Controllers (`tasks/`)
FreeRTOS tasks implementing system behaviors and coordination.

**[Full Task Documentation →](tasks/README.md)**

- **MotorTask**: FSM-based motor control (velocity, position, camera, test modes)
- **EncoderTask**: High-frequency encoder sampling and velocity estimation
- **CameraTask**: LED blob detection and visual servoing error computation
- **ControllerTask**: Global mode management and task coordination
- **UITask**: Web server, WebSocket telemetry, command processing
- **IMU_TASK**: Orientation sensing and motion data acquisition

### 3. FSM Framework (`fsm/`)
Reusable finite state machine implementation for behavioral control.

**[Full FSM Documentation →](fsm/README.md)**

- **FSM**: Execute-driven state machine engine
- **State**: Base class for state definitions
- **Usage**: Task behaviors modeled as state transitions

## 🔄 Data Flow

```
┌─────────────┐         ┌──────────────┐         ┌─────────────┐
│   Camera    │────────▶│  CameraTask  │────────▶│  MotorTask  │
│   (OV5640)  │  Image  │  (CV Loop)   │ Error   │  (Control)  │
└─────────────┘         └──────────────┘         └─────────────┘
                                                         │
                                                         │ PWM
                                                         ▼
┌─────────────┐         ┌──────────────┐         ┌─────────────┐
│   Encoder   │◀────────│ EncoderTask  │◀────────│  DRV8833    │
│   (PCNT)    │Position │ (Feedback)   │         │  (Driver)   │
└─────────────┘         └──────────────┘         └─────────────┘
                               │
                               │ Telemetry
                               ▼
┌─────────────┐         ┌──────────────┐         ┌─────────────┐
│   WiFi      │◀────────│   UITask     │◀────────│ Controller  │
│  (WebUI)    │WebSocket│  (Server)    │Commands │   Task      │
└─────────────┘         └──────────────┘         └─────────────┘
```

## 💻 Development Guide

### Building the Project
```bash
# Full build
pio run

# Upload firmware
pio run --target upload

# Upload web interface files
pio run --target uploadfs

# Serial monitor
pio device monitor
```

### Code Style
- **Doxygen Comments**: All functions, classes, and files documented
- **Naming Conventions**: 
  - Classes: `PascalCase`
  - Functions: `snake_case()`
  - Constants: `UPPER_SNAKE_CASE`
- **Indentation**: 4 spaces (no tabs)
- **Line Length**: 100 characters recommended

### Adding New Features

#### 1. New Hardware Driver
```cpp
// In src/hardware/NewDriver.h
class NewDriver {
public:
    NewDriver(int pin);
    void init();
    int read();
};

// Register in main.cpp
NewDriver driver(GPIO_PIN);
driver.init();
```

#### 2. New Task
```cpp
// In src/tasks/NewTask.h
class NewTask : public Task {
public:
    NewTask(const char* name, uint32_t stack, uint8_t priority);
    void run(void* params) override;
};

// Create in main.cpp
NewTask* newTask = new NewTask("NewTask", 4096, 3);
newTask->begin();
```

#### 3. New FSM State
```cpp
// Define state function
int new_state_execute() {
    // State behavior
    return NEXT_STATE_ID;  // or same ID to stay
}

// Add to state table
State newState(STATE_ID, "NEW_STATE", new_state_execute);
State* states[] = {&state0, &newState, ...};
```

## 🐛 Debugging

### Serial Debugging
```cpp
Serial.println("Debug message");
Serial.printf("Value: %d\n", variable);
```

### Task Monitoring
```cpp
// In any task
void run(void* params) {
    while (1) {
        Serial.printf("[%s] Running at %lu\n", 
                      pcTaskGetName(NULL), 
                      xTaskGetTickCount());
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
```

### Common Issues

**Task Crashes**
- Increase stack size in task constructor
- Check for stack overflow with `uxTaskGetStackHighWaterMark()`
- Verify no blocking operations in ISRs

**Queue Overflows**
- Increase queue depth when creating
- Check consumer task is reading fast enough
- Add queue full error handling

**Timing Issues**
- Use `vTaskDelay()` not `delay()` in tasks
- Verify task priorities are appropriate
- Check for priority inversion on shared resources

## 📊 Performance Metrics

- **Control Loop**: 100 Hz (10ms period)
- **Camera Processing**: ~15 FPS
- **Encoder Sampling**: 1000 Hz
- **WebSocket Updates**: 10 Hz
- **CPU Utilization**: ~60% average (dual-core)
- **Memory Usage**: ~180KB RAM (PSRAM for camera)

## 🔧 Configuration

### WiFi Settings
Edit `src/tasks/UITask.cpp`:
```cpp
const char* ssid = "YourNetworkName";
const char* password = "YourPassword";
```

### PID Tuning
Edit `src/main.cpp`:
```cpp
// Position control
pidKp = 0.5f;
pidKi = 0.0f;
pidKd = 0.1f;

// Camera tracking
pidKp_cam = 0.3f;
pidKi_cam = 0.0f;
pidKd_cam = 0.05f;
```

### Camera Detection
Edit `src/tasks/CameraTask.cpp`:
```cpp
// HSV thresholds for LED detection
int hue_min = 160;
int hue_max = 180;
int sat_min = 100;
int val_min = 100;
```

## 📚 API Documentation

Complete Doxygen-generated API documentation is available at:
**[docs/html/index.html](../docs/html/index.html)**

Key documentation sections:
- Class hierarchy and inheritance
- Function prototypes and parameters
- File dependencies and includes
- Global variables and defines

## 🧪 Testing

### Unit Testing
(Future: Add unit test framework)

### Integration Testing
1. **Motor Test Mode**: Verify PWM outputs and direction control
2. **Encoder Test**: Check position counting and velocity calculation
3. **Camera Test**: Validate LED detection accuracy
4. **Network Test**: Confirm WiFi connectivity and WebSocket streaming

### System Validation
- Run full tracking demo
- Verify mode transitions
- Check error handling and recovery
- Measure control loop timing

## 🔐 Safety Features

- **Watchdog Timer**: Reset on task deadlock
- **Motor Current Limiting**: DRV8833 thermal protection
- **Bounds Checking**: Encoder position limits
- **Command Validation**: Web interface input sanitization
- **Fault Recovery**: Automatic restart on critical errors

## 📞 Support

For software architecture questions, implementation details, or bug reports:
- Review existing [Doxygen documentation](../docs/html/index.html)
- Check [FSM design patterns](fsm/FSM.md)
- See specific subsystem READMEs:
  - [Hardware Drivers](hardware/README.md)
  - [Task Controllers](tasks/README.md)
  - [FSM Framework](fsm/README.md)

## 🚀 Future Enhancements

Potential improvements and extensions:
- [ ] Kalman filter for sensor fusion
- [ ] Multi-target tracking support
- [ ] Machine learning-based detection
- [ ] Bluetooth remote control
- [ ] SD card data logging
- [ ] OTA firmware updates
- [ ] Enhanced calibration routines
