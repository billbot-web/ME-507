# AI Assistant Instructions for MotorEncoder_Driver

## Project Overview
This is an ESP32-based motor control project using PlatformIO that combines:
- DRV883 motor driver control using dual PWM pins (IN1/IN2)
- Quadrature encoder reading using ESP32's PCNT (Pulse Counter) hardware
- Real-time telemetry output via serial

## Key Components

### Motor Driver (`DRV883` class)
- Uses ESP32's LEDC PWM for variable speed control
- Effort range: -100 to +100 (%)
- Pin mapping: IN1/IN2 control direction via PWM
- Default config: 20kHz PWM, 10-bit resolution

### Encoder (`Encoder` class) 
- Hardware quadrature decoder using ESP32 PCNT
- X4 mode (counts all edges of both channels)
- 16-bit counter range (-32768 to +32767)
- Velocity smoothing with 5-sample window
- Units: counts/microsecond (multiply by 1e6 for counts/sec)

## Project-Specific Constants
Current setup in `main.cpp`:
```cpp
// Encoder characteristics
// - Base: 16 pulses/rev (motor shaft)
// - Mode: X2 (counting A edges only) = 32 counts/rev
// - Gear ratio: 70:1
static constexpr float COUNTS_PER_REV = 2240.0f;  // 32 * 70
```

## Build & Debug Workflow
- Platform: ESP32 (FireBeetle32 board)
- Serial monitor: 115200 baud
- Telemetry format:
  ```
  ENC dC=<delta_counts> cps=<counts_per_sec> rpm=<output_shaft_rpm> dir=<FWD|REV|STOP> pos=<absolute_position>
  ```

## Development Patterns
1. Hardware configuration is done in constructors + begin() methods
2. Update encoder readings at fixed intervals (recommended: 1-10ms)
3. Position is accumulated internally, velocity uses smoothed window
4. Pin assignments must use ESP32 PCNT-capable GPIOs for encoder

## Integration Points
- Serial output for telemetry (115200 baud)
- PWM outputs: DRV883 IN1/IN2 pins
- Encoder inputs: ESP32 PCNT-capable pins only

## File Structure
- `src/DRV883.h/.cpp`: Motor driver interface
- `src/Encoder.h/.cpp`: PCNT-based encoder reader
- `src/main.cpp`: Example usage and telemetry
- `platformio.ini`: Project & build configuration