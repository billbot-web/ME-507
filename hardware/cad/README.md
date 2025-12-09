# Mechanical Design - CAD Files

This directory contains all mechanical design files for the SportTrackr pan-tilt tracking system, including SolidWorks assemblies, part models, and 3D-printable components.

## 📂 Directory Structure

```
cad/
├── ElectricalParts/        # Electronics housing and mounts
├── MechanicalParts/        # Mechanical components (gears, shafts, bearings)
├── Phone_Assembly/         # Camera/phone mounting system
├── Printed Parts/          # 3D-printable components
│   ├── STLs/               # Print-ready STL files
│   └── GCODE/              # Pre-sliced G-code files
└── SportTrackr_Assembly.SLDASM  # Complete system assembly
```

## 🔩 Key Components

### Printed Parts
All 3D-printed parts are designed for FDM printing with PLA material:

- **Base.SLDPRT** - Main mounting platform
- **Tilt_Mount.SLDPRT** - Tilt axis motor mount
- **Tilt_Camera_Mount.SLDPRT** - Camera bracket for tilt mechanism
- **Test_motorMount.SLDPRT** - Motor mounting bracket
- **IRFilterMount.SLDPRT** - IR filter holder for camera
- **Filtermountcover.SLDPRT** - Protective cover for filter
- **Housing2.SLDPRT** - Electronics enclosure
- **base mount.SLDPRT** - Base attachment point

**Print Settings:**
- Layer Height: 0.15mm
- Nozzle: 0.4mm
- Infill: 20-30%
- Supports: Required for some parts (see STL orientation)

### Mechanical Components

#### Power Transmission
- **30 THEETH 32MM GEAR.SLDPRT** - Main drive gear (30T, 32mm OD)
- **GT2 Pulley 20 Tooth.SLDPRT** - Timing belt pulley (20T, 6mm bore)
- **GT2_20T-sh8.SLDPRT** - Timing belt pulley (20T, 8mm bore)

#### Shafts & Bearings
- **Shaft.SLDPRT** - Main drive shaft (custom machined)
- **Shaft_Coupler.SLDPRT** - Flexible shaft coupling
- **FlangedShaft_Collar.SLDPRT** - Shaft retention collar
- **8mm Bearing Mount.SLDPRT** - Bearing housing

### Electrical Components
CAD models for electronics integration:

- **ESP32.SLDPRT** - ESP32-WROVER module model
- **ESP32_BAT.SLDPRT** - ESP32 with battery pack
- **OV5640.SLDPRT** - Camera module model
- **Motor.SLDPRT** - DC gearmotor model
- **MotorBat.SLDPRT** - Motor with power connections
- **sport_trackr_PCB.SLDASM** - Custom PCB assembly

### Phone/Camera Assembly
Alternative mounting system for smartphone camera:

- **Phone_Assembly.SLDASM** - Complete phone mounting assembly
- **Phone_Mount.SLDPRT** - Primary phone bracket
- **Phone_Mount2.SLDPRT** - Secondary mounting option
- **phone.SLDPRT** - Phone reference model

## 🔧 Assembly Instructions

1. **Print all parts** from `Printed Parts/STLs/` directory
2. **Mechanical assembly**:
   - Install bearings in bearing mounts
   - Insert shaft through tilt mechanism
   - Attach motors to motor mounts
   - Install gears/pulleys on motor shafts
   - Secure with shaft collars
3. **Electronics integration**:
   - Mount ESP32 board in housing
   - Attach camera to camera mount
   - Install PCB in base enclosure
   - Route wiring through cable channels
4. **Final assembly**:
   - Attach tilt mechanism to base
   - Mount pan motor to base plate
   - Connect all electrical connections
   - Secure covers and protective housings

## 📐 Design Specifications

- **Total Height**: ~250mm (assembled)
- **Base Footprint**: ~150mm × 150mm
- **Tilt Range**: ±45° (mechanical stops)
- **Pan Range**: 360° continuous rotation
- **Weight**: ~800g (with electronics)
- **Material**: PLA plastic + aluminum shaft

## 🛠️ Required Hardware

### Fasteners
- M3 screws (various lengths: 6mm, 8mm, 12mm, 20mm)
- M3 heat-set inserts (20×)
- M4 screws for motor mounting (8×)
- M2 screws for PCB mounting (4×)

### Mechanical
- 8mm diameter steel shaft (150mm length)
- 608 bearings (2×)
- GT2 timing belt (200mm)
- Shaft couplers (2×)

### Electronics (see [PCB README](../pcb/README.md))

## 📄 File Formats

- **.SLDPRT** - SolidWorks part files (editable)
- **.SLDASM** - SolidWorks assembly files (editable)
- **.SLDDRW** - SolidWorks drawing files (fabrication reference)
- **.STL** - Stereolithography files (3D printing)
- **.STEP** - Universal CAD exchange format (available on request)

## 🖨️ 3D Printing Notes

Pre-sliced G-code files are available in `Printed Parts/GCODE/` for Prusa Mini+ printer. For other printers, import STL files into your slicer with these recommended settings:

- **Material**: PLA or PETG
- **Layer Height**: 0.15mm (quality) or 0.2mm (speed)
- **Wall Count**: 3-4 perimeters
- **Infill**: 20-30% gyroid or cubic
- **Supports**: Tree supports for overhangs >45°
- **Adhesion**: Brim for tall/narrow parts

Total print time: ~24 hours for complete set

## 🔄 Modification Guidelines

When modifying designs:
1. Maintain mounting hole patterns for compatibility
2. Preserve critical dimensions for motor/bearing fits
3. Update assembly file after part changes
4. Regenerate STL files after edits
5. Test-fit before full assembly

## 📞 Design Contact

For questions about mechanical design or part modifications, contact the mechanical design team member or refer to the main project repository.
