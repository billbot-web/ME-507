# MotorEncoder_Driver

ESP32 DRV883 motor driver example with optional encoder support and a small FSM-driven MotorTask.

Build & flash (PlatformIO):

```powershell
# build
pio run
# upload
pio run -t upload
# monitor serial
pio device monitor -b 115200
```

Notes
- Project targets an ESP32 (FireBeetle32) board using PlatformIO.
- Motor driver is in `src/DRV883.*` and a simple FSM-based `MotorTask` is in `src/MotorTask.*`.
