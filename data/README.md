# Web User Interface

This directory contains all web interface assets for the SportTrackr remote control and telemetry system. The UI provides real-time monitoring, manual control, and system configuration through a responsive web interface served by the ESP32.

## 📂 Directory Structure

```
data/
├── index.html         # Main web application
├── css/
│   └── style.css     # Styling and layout
└── js/
    └── app.js        # Client-side JavaScript logic
```

## 🌐 Overview

The SportTrackr web interface is a single-page application (SPA) that communicates with the ESP32 over WiFi using HTTP for initial page load and WebSockets for real-time bidirectional communication.

### Features
- **Live Telemetry Dashboard**: Real-time display of motor positions, velocities, IMU data
- **Mode Selection**: Switch between tracking, teleop, motor test, and calibration modes
- **Manual Control**: Velocity joystick control for pan/tilt motors
- **Camera Stream**: Live video feed from OV5640 camera (JPEG over HTTP)
- **PID Configuration**: Adjust controller gains on-the-fly
- **System Status**: Connection state, error messages, diagnostics

## 🎨 User Interface Layout

### Header Section
- System title and logo
- Connection status indicator
- Mode selection dropdown

### Main Dashboard (Grid Layout)

#### Left Panel - Camera View
- Live video stream display
- Crosshair overlay showing target position
- Frame rate and resolution info
- LED detection visual feedback

#### Center Panel - Telemetry
Real-time numeric displays:
- **Motor Positions**: Pan/tilt encoder counts
- **Motor Velocities**: Current speed in counts/sec
- **IMU Orientation**: Euler angles (roll, pitch, yaw)
- **Tracking Error**: X/Y pixel error from target
- **System Time**: Uptime and cycle count

#### Right Panel - Controls
- **Mode Buttons**: 
  - 🎯 TRACKER - Auto tracking
  - 🕹️ TELEOP - Manual control
  - 🔧 MOTOR_TEST - Diagnostics
  - 📐 CALIBRATE - Homing
- **Joystick Control**: Touch/mouse velocity input
- **PID Tuning Sliders**: Kp, Ki, Kd adjustment
- **Emergency Stop**: Immediate motor shutdown

### Footer Section
- WebSocket connection state
- Data rate (messages/sec)
- Latency indicator
- Help/documentation link

## 🔌 Communication Protocol

### WebSocket Message Format

#### Client → Server (Commands)
```json
{
    "type": "command",
    "action": "set_mode",
    "value": "TRACKER"
}

{
    "type": "joystick",
    "pan_velocity": 50,
    "tilt_velocity": -30
}

{
    "type": "pid_update",
    "kp": 0.5,
    "ki": 0.0,
    "kd": 0.1
}
```

#### Server → Client (Telemetry)
```json
{
    "type": "telemetry",
    "timestamp": 1234567890,
    "pan_position": 2000,
    "tilt_position": -500,
    "pan_velocity": 120,
    "tilt_velocity": -80,
    "imu_roll": 2.5,
    "imu_pitch": -1.2,
    "imu_yaw": 45.7,
    "tracking_error_x": 15,
    "tracking_error_y": -22,
    "current_mode": "TRACKER"
}
```

### HTTP Endpoints

- **GET /** - Serve main HTML page
- **GET /style.css** - Serve stylesheet
- **GET /app.js** - Serve JavaScript
- **GET /stream** - MJPEG camera stream
- **GET /status** - JSON system status
- **POST /config** - Update configuration

## 💻 JavaScript Architecture (`app.js`)

### Core Components

#### WebSocket Manager
```javascript
class WebSocketManager {
    connect()           // Establish WebSocket connection
    send(message)       // Send JSON command
    onMessage(handler)  // Register message callback
    reconnect()         // Auto-reconnect on disconnect
}
```

#### Telemetry Display
```javascript
class TelemetryDisplay {
    update(data)        // Refresh all UI elements
    formatValue(val)    // Format numbers for display
    updateGraph(data)   // Update real-time plots
}
```

#### Control Input
```javascript
class JoystickControl {
    init(canvas)        // Initialize touch canvas
    onMove(callback)    // Register movement handler
    getVelocity()       // Get current X/Y values
    reset()             // Return to center
}
```

#### Mode Controller
```javascript
class ModeController {
    setMode(mode)       // Request mode change
    getCurrentMode()    // Get active mode
    validateTransition() // Check if mode switch allowed
}
```

### Event Handlers

```javascript
// WebSocket connection established
ws.onopen = () => {
    console.log("Connected to SportTrackr");
    startTelemetry();
};

// Incoming telemetry data
ws.onmessage = (event) => {
    const data = JSON.parse(event.data);
    updateDashboard(data);
};

// User clicks mode button
document.getElementById('mode-tracker').onclick = () => {
    sendCommand({type: 'set_mode', value: 'TRACKER'});
};

// Joystick movement
joystick.onMove = (x, y) => {
    sendCommand({
        type: 'joystick',
        pan_velocity: x * 100,
        tilt_velocity: y * 100
    });
};
```

## 🎨 Styling (`style.css`)

### Design System

**Color Palette:**
- Primary: `#2196F3` (Blue)
- Success: `#4CAF50` (Green)
- Warning: `#FF9800` (Orange)
- Danger: `#F44336` (Red)
- Background: `#1E1E1E` (Dark)
- Text: `#FFFFFF` (White)

**Typography:**
- Headings: `Roboto, sans-serif`
- Body: `Arial, sans-serif`
- Monospace: `Consolas, monospace`

**Layout:**
- CSS Grid for main dashboard
- Flexbox for controls and buttons
- Responsive breakpoints at 768px, 1024px

### Key Classes

```css
.dashboard-grid {
    display: grid;
    grid-template-columns: 1fr 1fr 1fr;
    gap: 20px;
}

.telemetry-card {
    background: #2C2C2C;
    border-radius: 8px;
    padding: 15px;
    box-shadow: 0 2px 4px rgba(0,0,0,0.3);
}

.mode-button {
    padding: 12px 24px;
    border-radius: 4px;
    transition: all 0.3s ease;
}

.mode-button.active {
    background: #4CAF50;
    box-shadow: 0 0 10px rgba(76, 175, 80, 0.5);
}
```

## 📱 Responsive Design

### Desktop (>1024px)
- Three-column grid layout
- Full camera stream size
- All controls visible simultaneously

### Tablet (768px - 1024px)
- Two-column grid
- Stacked control panels
- Reduced camera resolution

### Mobile (<768px)
- Single column layout
- Collapsible panels
- Touch-optimized controls
- Simplified joystick

## 🚀 Deployment

### Upload to ESP32
```bash
# Using PlatformIO
pio run --target uploadfs

# Or using esptool
esptool.py --chip esp32 --port COM4 write_flash 0x291000 spiffs.bin
```

### File System
Files are stored in ESP32 SPIFFS (SPI Flash File System):
- Total Size: 1.5MB
- Used: ~50KB (HTML/CSS/JS)
- Available: ~1.45MB for future assets

### Optimization
- Minify JavaScript/CSS for production
- Compress images (if added)
- Use gzip compression on server
- Cache static assets

## 🔧 Configuration

### WiFi Settings
Set in `src/tasks/UITask.cpp`:
```cpp
const char* ssid = "YourNetwork";
const char* password = "YourPassword";
IPAddress local_IP(192, 168, 1, 100);  // Optional static IP
```

### WebSocket Port
Default: 80 (HTTP) with WebSocket upgrade

### Access URL
After connecting to WiFi:
```
http://192.168.1.100/         (if static IP)
http://sporttrackr.local/     (if mDNS enabled)
```

## 🐛 Debugging

### Browser Console
```javascript
// Enable verbose logging
localStorage.setItem('debug', 'true');

// Monitor WebSocket messages
ws.onmessage = (e) => {
    console.log('RX:', e.data);
};
```

### Common Issues

**WebSocket won't connect:**
- Check ESP32 IP address
- Verify firewall settings
- Ensure WebSocket port is open
- Try different browser

**Telemetry not updating:**
- Check WebSocket connection state
- Verify JSON parsing (check console)
- Confirm UITask is running on ESP32
- Check telemetry update rate in firmware

**Camera stream blank:**
- Verify camera initialization
- Check HTTP endpoint `/stream`
- Ensure sufficient PSRAM for JPEG buffer
- Try lower resolution setting

## 🎯 Performance

### Metrics
- **Initial Load Time**: <2 seconds
- **WebSocket Latency**: 20-50ms
- **Telemetry Update Rate**: 10 Hz (100ms interval)
- **UI Refresh Rate**: 60 FPS (CSS animations)
- **Camera FPS**: 10-15 (depends on resolution)

### Optimization Tips
- Throttle joystick events (max 20 Hz)
- Batch telemetry updates
- Use CSS transforms for smooth animations
- Debounce PID slider inputs

## 🔐 Security Considerations

### Current Implementation
- **No authentication** - Open access on local network
- **No encryption** - Plain HTTP/WebSocket
- **No input validation** - Trust client data

### Future Enhancements
- [ ] Basic authentication (username/password)
- [ ] HTTPS/WSS with self-signed certificate
- [ ] Command rate limiting
- [ ] Input sanitization on server side
- [ ] Session management
- [ ] CORS configuration

## 📚 Related Documentation

- [UITask Implementation](../src/tasks/README.md#uitask) - Server-side code
- [Software Architecture](../src/README.md) - Overall system design
- [System Overview](../README.md) - Project documentation

## 🚀 Future Features

Planned UI enhancements:
- [ ] Real-time plotting (position/velocity graphs)
- [ ] Recording and playback of tracking sessions
- [ ] Multiple camera view support
- [ ] Advanced PID auto-tuning wizard
- [ ] Diagnostic logs viewer
- [ ] Mobile app (React Native or Flutter)
- [ ] Dark/light theme toggle
- [ ] Multi-language support
