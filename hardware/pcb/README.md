# Electronic Design - PCB Files

This directory contains all electronic design files for the SportTrackr system PCB, including schematics, board layout, and manufacturing files.

## 📂 Files

- **sport_trackr_copy.kicad_pro** - KiCad project file
- **sport_trackr_copy.kicad_sch** - Schematic diagram
- **sport_trackr_copy.kicad_pcb** - PCB layout (board design)
- **sport_trackr_copy.kicad_prl** - Project local settings
- **READ_ME** - Original design notes

## 🔌 PCB Overview

The SportTrackr PCB is a custom-designed control board that interfaces the ESP32-WROVER module with all peripheral hardware including motor drivers, encoders, camera, and IMU sensor.

### Board Specifications
- **Dimensions**: TBD (see KiCad layout)
- **Layers**: 2-layer PCB
- **Thickness**: 1.6mm standard FR4
- **Copper Weight**: 1oz (35μm)
- **Finish**: HASL or ENIG

## 🧩 System Components

### Microcontroller
- **ESP32-WROVER Module**
  - Dual-core Xtensa LX6 @ 240MHz
  - 4MB PSRAM for camera buffering
  - WiFi 802.11 b/g/n
  - Bluetooth 4.2 BLE

### Motor Control
- **2× DRV8833 Dual H-Bridge Drivers**
  - Per-channel current rating: 1.5A continuous, 2A peak
  - PWM frequency: 20kHz
  - Sleep/coast control pins
  - Thermal shutdown protection
  - Fault status output

### Sensors

#### Quadrature Encoders (2×)
- **Interface**: ESP32 PCNT peripheral
- **Resolution**: 64 CPR (counts per revolution)
- **Total Resolution**: 4480 counts/rev with 70:1 gearbox
- **Channels**: A, B quadrature signals per encoder
- **Voltage**: 3.3V logic level

#### Camera Module
- **OV5640 5MP Camera**
  - Interface: SCCB (I2C) + parallel data
  - Resolution: 2592×1944 (5MP), configurable
  - Frame Rate: ~15 FPS at 640×480
  - Features: Autofocus, JPEG compression
  - Power: 3.3V digital, 2.8V analog

#### IMU Sensor
- **Adafruit BNO055**
  - 9-DOF sensor fusion (accel + gyro + mag)
  - Interface: I2C (address 0x28 or 0x29)
  - Output: Quaternions, Euler angles, vectors
  - Calibration: Automatic sensor calibration
  - Power: 3.3V

### Power Supply
- **Input**: 7-12V DC (barrel jack or screw terminal)
- **5V Regulator**: Buck converter for motor power
- **3.3V Regulator**: LDO for ESP32 and logic
- **Current Capacity**: 3A total
- **Protection**: Reverse polarity, overcurrent

### Connectors
- **Motor Outputs**: 2× screw terminal blocks (pan & tilt)
- **Encoder Inputs**: 2× 0.1" pin headers (4 pins each)
- **Camera Connector**: FFC/FPC connector (24-pin)
- **IMU Interface**: I2C header or JST connector
- **Programming**: Micro-USB (ESP32 native USB-to-serial)
- **Debug**: UART header for serial monitoring

## 📐 Pin Assignments

### Motor Control
| Motor | IN1 Pin | IN2 Pin | PWM Pin | Sleep Pin |
|-------|---------|---------|---------|-----------|
| Pan   | GPIO32  | GPIO33  | GPIO25  | GPIO26    |
| Tilt  | GPIO14  | GPIO27  | GPIO13  | GPIO12    |

### Encoders
| Encoder | Channel A | Channel B | PCNT Unit |
|---------|-----------|-----------|-----------|
| Pan     | GPIO34    | GPIO35    | PCNT_0    |
| Tilt    | GPIO36    | GPIO39    | PCNT_1    |

### Camera (OV5640)
- **SCCB**: SDA=GPIO21, SCL=GPIO22
- **Data**: D0-D7 on GPIO4-5, 18-19, 36-39
- **Control**: VSYNC=GPIO23, HREF=GPIO25, PCLK=GPIO22

### IMU (BNO055)
- **I2C**: SDA=GPIO21, SCL=GPIO22 (shared with camera)
- **INT Pin**: GPIO15 (optional interrupt)
- **RESET**: GPIO2 (optional reset control)

## 🛠️ Design Software

### KiCad (Version 7.0+)
The PCB is designed in KiCad, an open-source electronics design automation suite.

**Opening the Project:**
```bash
# Install KiCad from https://www.kicad.org/
# Open KiCad → File → Open Project
# Select sport_trackr_copy.kicad_pro
```

**Required Symbol Libraries:**
- ESP32 module symbols (custom or contributed)
- DRV8833 motor driver (Texas Instruments)
- Standard passives (resistors, capacitors)
- Connectors (JST, screw terminals, headers)

## 🏭 Manufacturing

### Gerber Files
To generate production files (Gerbers + drill files):
1. Open PCB layout in KiCad PCB Editor
2. File → Plot
3. Select Gerber output format
4. Include layers: F.Cu, B.Cu, F.Mask, B.Mask, F.Silkscreen, B.Silkscreen, Edge.Cuts
5. Generate drill files separately
6. Zip all files for manufacturer

### Recommended Manufacturers
- **JLCPCB** - Low cost, fast turnaround
- **PCBWay** - Good quality, assembly services
- **OSH Park** - USA-based, purple boards
- **ALLPCB** - Economical for prototypes

### Bill of Materials (BOM)
Major components (see full BOM in KiCad):
- 1× ESP32-WROVER module
- 2× DRV8833 motor driver IC (HTSSOP package)
- 1× AMS1117-3.3 LDO voltage regulator
- 1× Buck converter module (5V)
- Passives: resistors, capacitors (0805 SMD)
- Connectors: screw terminals, pin headers, FFC connector

### Assembly Notes
- ESP32 module requires careful hand-soldering or reflow
- DRV8833 ICs have thermal pad - ensure proper grounding
- Test all power rails before connecting peripherals
- Program/test ESP32 before final assembly

## 🧪 Testing & Validation

### Power-On Tests
1. ✅ Check 3.3V rail voltage
2. ✅ Check 5V rail voltage
3. ✅ Verify no short circuits
4. ✅ Test USB programming interface

### Peripheral Tests
1. ✅ Motor driver output (no-load PWM)
2. ✅ Encoder pulse counting
3. ✅ Camera SCCB communication (I2C scan)
4. ✅ IMU I2C communication (device ID read)

### Functional Tests
1. ✅ Motor velocity control
2. ✅ Encoder position feedback
3. ✅ Camera frame capture
4. ✅ IMU orientation data
5. ✅ WiFi connectivity

## 🔧 Troubleshooting

### Common Issues

**Motor not spinning:**
- Check PWM signal with oscilloscope
- Verify motor driver sleep pin is HIGH
- Measure motor supply voltage (should be 5V)
- Check for thermal shutdown on DRV8833

**Encoder not counting:**
- Verify 3.3V supply to encoder
- Check A/B signal integrity (use scope)
- Confirm ESP32 PCNT configuration
- Ensure proper pull-up resistors

**Camera not detected:**
- Verify I2C communication (use I2C scanner)
- Check camera module power (3.3V)
- Confirm FFC cable connection
- Test with known-good camera module

**IMU no response:**
- Check I2C address (0x28 default, 0x29 alternate)
- Verify I2C pull-up resistors (2.2kΩ recommended)
- Ensure proper power supply (3.3V)
- Try software reset via RESET pin

## 📄 Schematic Overview

### Power Distribution
- 7-12V input → 5V buck → Motor drivers, camera analog
- 5V → 3.3V LDO → ESP32, sensors, logic

### Signal Routing
- Motor PWM signals use ESP32 LEDC peripheral
- Encoders connect to ESP32 PCNT hardware counters
- Camera uses ESP32 parallel interface (I2S DMA)
- IMU/Camera share I2C bus (GPIO21/22)

### Protection Features
- Reverse polarity diode on power input
- Bypass capacitors on all power rails (0.1μF + 10μF)
- ESD protection on USB port
- Thermal relief on motor driver ground pads

## 🔄 Revision History

- **v1.0** - Initial design with all core functionality
- Future revisions will be documented here

## 📞 Support

For PCB design questions, manufacturing issues, or electrical modifications, contact the electronics design team member or open an issue in the project repository.

## 📚 Related Documentation

- [Software Documentation](../../src/README.md) - Firmware implementation
- [Hardware Drivers](../../src/hardware/README.md) - Low-level peripheral code
- [Mechanical Integration](../cad/README.md) - Physical mounting and assembly
