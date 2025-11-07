# MotorEncoder_Driver

ESP32-based motor control system using DRV883 driver with FreeRTOS task management and FSM implementation.

## Features

- **DRV883 Motor Control**
  - Variable speed control using ESP32's LEDC PWM (20kHz, 10-bit)
  - Bidirectional control (-100% to +100% effort)
  - Dual PWM pin control (IN1/IN2) for direction

- **FreeRTOS Integration**
  - Motor control runs in dedicated task
  - Core pinning for reliable timing
  - Task-safe communication

- **FSM Implementation**
  - WAIT/RUN state machine
  - Configurable timing for each state
  - Clean separation of control logic

## Hardware Setup

- **Board**: FireBeetle32 (ESP32)
- **Motor Driver**: DRV883
  - IN1/IN2: PWM direction control
  - Configuration: 20kHz PWM, 10-bit resolution

## Project Structure

```
src/
├── DRV883.cpp/.h     # Motor driver interface
├── MotorTask.cpp/.h  # FSM implementation
└── main.cpp          # Application entry & task creation
```

## Building & Flashing

Using PlatformIO:

```powershell
# Build project
pio run

# Upload to ESP32
pio run -t upload

# Monitor serial output
pio device monitor -b 115200
```

## Class Overview

### DRV883
Motor driver interface handling PWM generation and direction control.
- `setEffort(float)`: Set motor effort (-100 to +100)
- Hardware PWM using ESP32's LEDC peripheral

### MotorTask
FreeRTOS task implementing a simple FSM for motor control.
- States: WAIT, RUN
- Configurable timing for each state
- Task-safe motor control

## Development Notes

- Project uses PlatformIO for build management
- FreeRTOS task runs on core 1 by default
- Default PWM frequency: 20kHz
- Serial monitoring at 115200 baud

ESP32 DRV883 motor driver example with optional encoder support and a small FSM-driven MotorTask.
`
Notes
- Project targets an ESP32 (FireBeetle32) board using PlatformIO.
- Motor driver is in `src/DRV883.*` and a simple FSM-based `MotorTask` is in `src/MotorTask.*`.
