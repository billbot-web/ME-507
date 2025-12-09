# SportTrackr: Camera-Guided Pan-Tilt Tracking System

An ESP32-based real-time tracking system that combines computer vision, motion control, and web telemetry to automatically track LED targets using a camera-guided pan-tilt mechanism.

![System Overview](docs/html/index.html)

## 🎯 Project Overview

SportTrackr is an embedded control system designed for automated visual tracking applications. The system uses an OV5640 camera module for target detection, dual DC motors with encoders for precise pan-tilt control, and an IMU for orientation sensing. All components are coordinated through a FreeRTOS-based task architecture with PID control loops for smooth and accurate tracking.

### Key Features

- **Autonomous LED Tracking**: Real-time computer vision processing for target detection and tracking
- **Multi-Mode Operation**: Automatic tracking, manual control, velocity mode, and position mode
- **Web Interface**: Live telemetry streaming and remote control via WiFi
- **Precision Control**: Closed-loop PID control with encoder feedback
- **Sensor Fusion**: 9-DOF IMU integration for enhanced stability
- **Modular Architecture**: FSM-based task design for easy extension and debugging

## 📁 Repository Structure

```
├── data/              # Web UI assets (HTML, CSS, JavaScript)
├── docs/              # Doxygen-generated documentation
├── src/               # Source code
│   ├── hardware/      # Hardware drivers (motors, encoders, camera, IMU)
│   ├── tasks/         # FreeRTOS task implementations
│   ├── fsm/           # Finite state machine framework
│   └── main.cpp       # System initialization and setup
└── hardware/          # Physical design files
    ├── cad/           # Mechanical CAD files (SolidWorks)
    └── pcb/           # PCB design files (KiCad)
```

## 🔧 System Components

### Hardware
- **MCU**: ESP32-WROVER (Dual-core, WiFi, 4MB PSRAM)
- **Camera**: OV5640 (5MP, autofocus, JPEG compression)
- **Motors**: 2× DC motors with 70:1 gearboxes
- **Drivers**: 2× DRV8833 dual H-bridge motor controllers
- **Encoders**: 2× Quadrature encoders (64 CPR)
- **IMU**: BNO055 (9-DOF: accelerometer, gyroscope, magnetometer)

### Software Stack
- **Framework**: Arduino/ESP-IDF on PlatformIO
- **RTOS**: FreeRTOS for multitasking
- **Web Server**: ESPAsyncWebServer for WiFi interface
- **Control**: Custom PID implementation with multiple tuning profiles

## 📚 Documentation

Detailed documentation is available for each subsystem:

- **[Mechanical Design](hardware/cad/README.md)** - CAD models, assemblies, and 3D-printed parts
- **[Electronic Design](hardware/pcb/README.md)** - PCB schematics, board layout, and wiring
- **[Software Architecture](src/README.md)** - Code organization and design patterns
  - [Hardware Drivers](src/hardware/README.md) - Low-level peripheral interfaces
  - [Task Controllers](src/tasks/README.md) - FreeRTOS task implementations
  - [FSM Framework](src/fsm/README.md) - State machine design pattern
- **[Web Interface](data/README.md)** - User interface and telemetry system
- **[API Reference](docs/html/index.html)** - Full Doxygen documentation

## 🚀 Getting Started

### Prerequisites
- PlatformIO IDE (VS Code extension recommended)
- ESP32 toolchain
- USB-to-Serial driver for ESP32

### Build and Upload
```bash
# Clone the repository
git clone https://github.com/billbot-web/ME-507.git
cd ME-507

# Build the project
pio run

# Upload to ESP32
pio run --target upload

# Upload filesystem (web UI)
pio run --target uploadfs

# Monitor serial output
pio device monitor
```

### Configuration
1. Update WiFi credentials in `src/tasks/UITask.cpp`
2. Adjust PID tuning parameters in `src/main.cpp` (see comments for guidance)
3. Calibrate camera detection thresholds in `src/tasks/CameraTask.cpp`

## 🎮 Operating Modes

The system supports four distinct operating modes:

1. **TRACKER**: Autonomous camera-based target tracking with PID control
2. **TELEOP**: Manual velocity control via web interface
3. **MOTOR_TEST**: Open-loop motor testing for diagnostics
4. **CALIBRATE**: Encoder homing and reference position setup

Modes are switched via the web interface or programmatically through the ControllerTask.

## 🔬 System Architecture

The system uses a modular FreeRTOS task architecture:

- **MotorTask** (2 instances): Multi-mode motor control with FSM-based state management
- **EncoderTask** (2 instances): High-frequency position/velocity measurement using ESP32 PCNT
- **ControllerTask**: System coordinator and mode manager
- **CameraTask**: Computer vision processing for LED detection
- **IMUTask**: Orientation sensing and sensor data fusion
- **UITask**: Web server, telemetry streaming, and user commands

Tasks communicate via FreeRTOS queues and share data through thread-safe mechanisms.

## 📊 Performance

- **Control Loop Rate**: 100 Hz (10ms task periods)
- **Camera Frame Rate**: ~10-15 FPS (JPEG compression)
- **Encoder Resolution**: 4480 counts/revolution (with gearbox)
- **Position Accuracy**: ±0.5° typical
- **Latency**: <50ms from detection to motor response

## 🤝 Contributors

- **Johnny Rourke** - Software & Integration
- **William Kuschman** - Electronics & PCB Design
- **Sam Nussman** - Mechanical Design & CAD

## 📄 License

This project was developed as part of ME 507 (Mechatronics) at California Polytechnic State University, San Luis Obispo.

## 🔗 Additional Resources

- [Doxygen Documentation](docs/html/index.html) - Complete API reference
- [FSM Design Pattern](src/fsm/FSM.md) - State machine implementation details
- [PID Tuning Guide](src/main.cpp) - Controller configuration instructions
