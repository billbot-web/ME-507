# Hardware Drivers

These Low-level drivers provide abstraction to easily and repeatably interface with multiple hardware peripherals using the ESP32. These drivers include applications for motor control, sensing, and vision.

## 📂 Files

- **DRV883.cpp/h** - DRV8833 dual H-bridge motor driver control
- **Encoder.cpp/h** - Quadrature encoder interface using ESP32 PCNT
- **OV5640_camera.cpp/h** - OV5640 camera module with JPEG capture
- **Adafruit_BNO055.cpp/h** - BNO055 9-DOF IMU sensor driver. This is an Open-source drive

## 🔌 DRV883 Motor Driver

### Overview
Interface to the DRV8833 dual H-bridge motor driver providing PWM speed control and direction control. This driver takes in four input parameters: the first two are the two pins on the ESP32 that you will use for PWM. To control the motor speed and direction, the other two are the corresponding PWM channels for the in1 and in2 pins, respectively. When setting the motor PWM effort, the input is meant in the range of -100 to 100. The PWM Effort represents the ratio of how often the motor is on compared with simply being turned on, and the negative is meant to represent turning in either direction. We currently clamp the input effort in code to the possible range, so you can input efforts outside this range, and it will automatically saturate the motor.

### Key Features
- **Dual PWM outputs** with configurable frequency (20kHz default)
- **Direction control** via IN1/IN2 pins (forward, reverse, brake, coast)
- <img width="1041" height="297" alt="image" src="https://github.com/user-attachments/assets/bd24d162-a991-4c89-a29a-43fec38e8193" />
[1]
The Disable pin is tied high internally. While the nSleep pin is shared for both motors so it is handled outside of the motor driver.
- **Current limiting** via hardware (1.5A continuous, 5A peak)

### API
```cpp
 DRV883(int in1, int in2, int ledcCh1, int ledcCh2,
                uint32_t pwmFreq = 20000, uint8_t resolutionBits = 10);
    void setEff(int effort);
    void stop();
    void brake();

```

### Usage Example
```cpp
DRV883 motor(GPIO32, GPIO33, GPIO25, GPIO26);

motor.set_seff(128);    // Half speed forward
motor.set_eff(-200);   // Fast reverse
motor.brake();           // Stop immediately
motor.coast();           // Let motor coast
motor.sleep(true);       // Power down driver
```

### Hardware Connections
| Motor | IN1   | IN2   | nSLP   |
|-------|-------|-------|--------|
| Pan   |GPIO02 |GPIO05 |GPIO13  |
| Tilt  |GPIO18 |GPIO23 |GPIO13  |
## Encoder Driver

### Overview
Quadrature encoder interface using ESP32 PCNT (Pulse Counter) peripheral for high-frequency, hardware-based position tracking.

### Key Features
- **Hardware counting** (no CPU overhead)
- **Quadrature decoding** (4× resolution enhancement)
- **Velocity estimation** via position differentiation
- **16-bit signed counter** with overflow handling
- **Up to 40MHz counting** frequency support
- **High Resolution** 32 counts per revolution of the motor, multiplied by the 70:1 Gear ratio to the output shaft, so 2240 counts per revolution for the output shaft.

### API
```cpp
class Encoder {
public:
    Encoder(int pinA, int pinB, pcnt_unit_t unit);
    void init();                    // Configure PCNT unit
    int32_t get_position();         // Read current count
    float get_velocity();           // Estimate speed (counts/sec)
    void zero();                   // Zero the counter
};
```

### Usage Example
```cpp
Encoder encoder(GPIO34, GPIO35, PCNT_UNIT_0);
encoder.init();

int32_t pos = encoder.get_position();
float vel = encoder.get_velocity();
encoder.zero();  // Re-home to zero
```

### Hardware Connections
| Encoder | Channel A | Channel B | PCNT Unit |
|---------|-----------|-----------|-----------|
| Pan     | GPIO34    | GPIO35    | UNIT_0    |
| Tilt    | GPIO14    | GPIO19    | UNIT_1    |

### Resolution Calculation
```
Base Resolution: 32 CPR (counts per revolution)
Gear Ratio: 70:1
Total: 32 × 70 = 2240 counts/motor_revolution
Final: 2240 counts per output shaft revolution (after gearbox)
```

## 📷 OV5640 Camera Driver

### Overview
Interface to OV5640 5-megapixel camera module with autofocus, providing JPEG-compressed image capture and LED blob detection. This allows for position and velocity tracking of both motors in the design. We only directly measure the angular position we move, so we need to divide by the time step between data points to determine the motor velocity. This can become inaccurate at low speeds. In order to smooth the velocity output, we also implemented a 5-position and 5 time-step buffer in order to average the encoder velocity over the past five data points to reduce noise. This also allows for closed-loop control on both motors.

### Key Features
- **High resolution**: Up to 2592×1944 (5MP)
- **Configurable formats**: JPEG, RGB565, YUV422
- **Autofocus control**: Software-triggered AF
- **JPEG compression**: Hardware JPEG encoder
- **LED detection**: HSV color-space blob tracking
- **Frame buffering**: PSRAM-based DMA transfers

### API
```cpp
class OV5640_camera {
public:
    OV5640_camera();
    bool init(framesize_t size);           // Initialize with resolution
    bool capture();                        // Capture single frame
    uint8_t* get_fb();                     // Get frame buffer pointer
    size_t get_fb_len();                   // Get frame size in bytes
    bool detect_led(int* x, int* y);       // Find LED centroid
    void set_hsv_threshold(int h_min, int h_max, 
                           int s_min, int v_min);
    void autofocus();                      // Trigger autofocus
};
```

### Usage Example
```cpp
OV5640_camera cam;
cam.init(FRAMESIZE_VGA);  // 640×480
cam.set_hsv_threshold(160, 180, 100, 100);  // Red LED

if (cam.capture()) {
    int led_x, led_y;
    if (cam.detect_led(&led_x, &led_y)) {
        Serial.printf("LED at (%d, %d)\n", led_x, led_y);
    }
    
    uint8_t* jpeg = cam.get_fb();
    size_t len = cam.get_fb_len();
    // Send JPEG data...
}
```

### Hardware Connections (SCCB + Parallel)
| Signal | GPIO | Description |
|--------|------|-------------|
| SDA    | 21   | I2C data (SCCB) |
| SCL    | 22   | I2C clock (SCCB) |
| D0-D7  | 4-5, 18-19, 36-39 | Parallel data bus |
| VSYNC  | 23   | Vertical sync |
| HREF   | 25   | Horizontal reference |
| PCLK   | 22   | Pixel clock |

### Detection Algorithm
1. Convert RGB → HSV color space
2. Threshold by hue/saturation/value ranges
3. Find largest contiguous blob
4. Calculate centroid (weighted average)
5. Return pixel coordinates (0,0 = center)

## 🧭 Adafruit_BNO055 IMU Driver

### Overview
Interface to BNO055 9-DOF absolute orientation sensor providing fused accelerometer, gyroscope, and magnetometer data.

### Key Features
- **Sensor fusion**: Onboard ARM Cortex M0+ processor
- **Absolute orientation**: Quaternion and Euler angle output
- **9 degrees of freedom**: 3-axis accel + gyro + mag
- **Automatic calibration**: Self-calibrating sensors
- **Fast I2C**: Up to 400kHz bus speed

### API
```cpp
class Adafruit_BNO055 {
public:
    Adafruit_BNO055(int32_t id, uint8_t address);
    bool begin();                          // Initialize sensor
    void setMode(adafruit_bno055_opmode_t mode);
    imu::Vector<3> getVector(vector_type_t type);
    uint8_t getCalibration();              // Get cal status (0-3)
    void getQuat(double* w, double* x, double* y, double* z);
    void getEuler(double* roll, double* pitch, double* yaw);
};
```

### Usage Example
```cpp
Adafruit_BNO055 bno(55, 0x28);  // ID=55, I2C addr=0x28
if (bno.begin()) {
    bno.setMode(OPERATION_MODE_NDOF);  // 9-DOF fusion
    
    // Read orientation
    double roll, pitch, yaw;
    bno.getEuler(&roll, &pitch, &yaw);
    
    // Check calibration (0=uncal, 3=fully calibrated)
    uint8_t cal = bno.getCalibration();
    if (cal >= 2) {
        Serial.println("IMU calibrated");
    }
    
    // Read raw sensors
    imu::Vector<3> accel = bno.getVector(VECTOR_ACCELEROMETER);
    imu::Vector<3> gyro = bno.getVector(VECTOR_GYROSCOPE);
}
```

### Hardware Connections
| Signal | GPIO | Description |
|--------|------|-------------|
| SDA    | 21   | I2C data (shared with camera) |
| SCL    | 22   | I2C clock (shared with camera) |
| INT    | 15   | Interrupt (optional) |
| RESET  | 2    | Reset (optional) |

### Coordinate System
- **X-axis**: Forward (pitch rotation)
- **Y-axis**: Left (roll rotation)
- **Z-axis**: Up (yaw rotation)
- **Euler Angles**: Roll (±180°), Pitch (±90°), Yaw (0-360°)

### Calibration Process
1. **Gyroscope**: Keep sensor stationary (auto-calibrates in ~1s)
2. **Accelerometer**: Rotate through all 6 faces (cube orientation)
3. **Magnetometer**: Move in figure-8 pattern (avoid magnetic interference)
4. **System**: Wait for all sensors to reach calibration level 3

## 🔧 Common Patterns

### Initialization Sequence
```cpp
void setup() {
    // Motor drivers
    motor_pan.init();
    motor_tilt.init();
    
    // Encoders
    encoder_pan.init();
    encoder_tilt.init();
    encoder_pan.set_filter(100);
    encoder_tilt.set_filter(100);
    
    // Camera
    if (!camera.init(FRAMESIZE_VGA)) {
        Serial.println("Camera init failed!");
    }
    
    // IMU
    if (!imu.begin()) {
        Serial.println("IMU init failed!");
    }
}
```

### Error Handling
```cpp
// Check for hardware faults
if (!motor.set_speed(speed)) {
    Serial.println("Motor driver fault!");
    motor.sleep(true);  // Disable driver
}

// Validate encoder readings
int32_t pos = encoder.get_position();
if (abs(pos) > MAX_POSITION) {
    Serial.println("Encoder out of range!");
    motor.brake();
}

// Verify camera capture
if (!camera.capture()) {
    Serial.println("Frame capture failed!");
    camera.init(FRAMESIZE_VGA);  // Re-init
}
```

### Resource Management
```cpp
// Power saving
motor.sleep(true);          // Disable motor driver
camera.deinit();            // Release camera resources
imu.setMode(SUSPEND_MODE);  // Low-power IMU mode

// Cleanup on task exit
motor.brake();
motor.sleep(true);
encoder.reset();
```

## Troubleshooting

### Motor Driver Issues
- **Motor not spinning**: Check PWM frequency, verify sleep pin HIGH
- **Thermal shutdown**: Reduce current, add heatsink, check for stall
- **Erratic behavior**: Add decoupling capacitors, check power supply

### Camera Issues
- **Init failure**: Check I2C address (0x3C), verify SCCB communication
- **Black frames**: Ensure HREF/VSYNC connected, check clock signals
- **Detection fails**: Tune HSV thresholds, improve lighting conditions

### IMU Issues
- **No response**: Verify I2C address (0x28 or 0x29), check power
- **Poor calibration**: Avoid magnetic interference, perform cal routine
- **Data jumps**: Filter outliers, use sensor fusion quaternions

## Related Documentation

- [Task Controllers](../tasks/README.md) - Higher-level task implementations
- [Software Architecture](../README.md) - System overview
- [PCB Design](../../hardware/pcb/README.md) - Hardware connections

## 🔗 Datasheets

- 1. [DRV8833 Datasheet](https://www.ti.com/lit/ds/symlink/drv8833.pdf)
- 2. [OV5640 Datasheet](https://cdn.sparkfun.com/datasheets/Sensors/LightImaging/OV5640_datasheet.pdf)
- 3. [BNO055 Datasheet](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bno055-ds000.pdf)
- 4. [ESP32 PCNT Documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/pcnt.html)
