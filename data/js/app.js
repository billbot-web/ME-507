// SporTrackr Control JavaScript

let ws = null;
let reconnectInterval = null;
let currentPage = 'mode-select';
let controlMode = 'velocity'; // 'velocity' or 'position'
let testControlMode = 'velocity'; // For motor test mode
let dpadInterval = null;
let dpadSpeed = 50;

// Page Navigation
function selectMode(mode) {
    if (mode === 'trackr') {
        showPage('page-trackr');
        sendCommand('mode:trackr');
        document.getElementById('current-mode').textContent = 'Tracker Mode';
    } else if (mode === 'teleop') {
        showPage('page-teleop');
        sendCommand('mode:teleop');
        document.getElementById('current-mode').textContent = 'Teleop Mode';
    } else if (mode === 'motor-test') {
        showPage('page-motor-test');
        sendCommand('mode:test');
        document.getElementById('current-mode').textContent = 'Motor Test';
    } else if (mode === 'calibrate') {
        showPage('page-calibrate');
        sendCommand('mode:calibrate');
        document.getElementById('current-mode').textContent = 'Calibrate';
    }
}

function goBack() {
    showPage('page-mode-select');
    sendCommand('mode:idle');
    sendCommand('imu:disable'); // Disable IMU in mode select
    document.getElementById('current-mode').textContent = 'Idle';
}

function showPage(pageId) {
    // Hide all pages
    const pages = document.querySelectorAll('.page');
    pages.forEach(page => page.classList.remove('active'));
    
    // Show selected page
    const selectedPage = document.getElementById(pageId);
    if (selectedPage) {
        selectedPage.classList.add('active');
        currentPage = pageId;
    }
}

// WebSocket Connection
function connectWebSocket() {
    ws = new WebSocket('ws://192.168.4.1/ws');
    
    ws.onopen = function() {
        document.getElementById('ws-status').textContent = 'Connected';
        document.getElementById('ws-status').className = 'connected';
        console.log('WebSocket connected');
        if (reconnectInterval) {
            clearInterval(reconnectInterval);
            reconnectInterval = null;
        }
    };
    
    ws.onmessage = function(event) {
        try {
            const data = JSON.parse(event.data);
            if (data.type === 'telemetry') {
                updateTelemetry(data);
            }
        } catch (e) {
            console.log('Received:', event.data);
        }
    };
    
    ws.onclose = function() {
        document.getElementById('ws-status').textContent = 'Disconnected';
        document.getElementById('ws-status').className = 'disconnected';
        console.log('WebSocket disconnected');
        
        // Auto-reconnect
        if (!reconnectInterval) {
            reconnectInterval = setInterval(connectWebSocket, 3000);
        }
    };
    
    ws.onerror = function(error) {
        console.log('WebSocket error:', error);
    };
}

function updateTelemetry(data) {
    // Update pan/tilt data across all pages
    const panVel = (data.pan_velocity || 0).toFixed(2);
    const panPos = (data.pan_position || 0).toFixed(2);
    const tiltVel = (data.tilt_velocity || 0).toFixed(2);
    const tiltPos = (data.tilt_position || 0).toFixed(2);
    
    // Mode select page
    updateElement('pan-velocity', panVel);
    updateElement('pan-position', panPos);
    updateElement('tilt-velocity', tiltVel);
    updateElement('tilt-position', tiltPos);
    
    // Tracker page
    updateElement('trackr-pan-velocity', panVel);
    updateElement('trackr-pan-position', panPos);
    updateElement('trackr-tilt-velocity', tiltVel);
    updateElement('trackr-tilt-position', tiltPos);
    
    // Teleop page
    updateElement('teleop-pan-velocity', panVel);
    updateElement('teleop-pan-position', panPos);
    updateElement('teleop-tilt-velocity', tiltVel);
    updateElement('teleop-tilt-position', tiltPos);
    
    // Motor test page
    updateElement('test-pan-velocity', panVel);
    updateElement('test-pan-position', panPos);
    updateElement('test-tilt-velocity', tiltVel);
    updateElement('test-tilt-position', tiltPos);
    
    // Calibrate page
    updateElement('cal-pan-position', panPos);
    updateElement('cal-tilt-position', tiltPos);
    
    // Update IMU data
    if (data.euler && data.euler.length === 3) {
        const eulerStr = data.euler[0].toFixed(1) + ', ' + data.euler[1].toFixed(1) + ', ' + data.euler[2].toFixed(1);
        updateElement('euler-value', eulerStr);
        updateElement('trackr-euler', eulerStr);
        updateElement('teleop-euler', eulerStr);
        updateElement('test-euler', eulerStr);
        updateElement('cal-euler', eulerStr);
        
        // Update detailed IMU fields for each page
        updateElement('trackr-imu-euler-x', data.euler[0].toFixed(2));
        updateElement('trackr-imu-euler-y', data.euler[1].toFixed(2));
        updateElement('trackr-imu-euler-z', data.euler[2].toFixed(2));
        updateElement('teleop-imu-euler-x', data.euler[0].toFixed(2));
        updateElement('teleop-imu-euler-y', data.euler[1].toFixed(2));
        updateElement('teleop-imu-euler-z', data.euler[2].toFixed(2));
        updateElement('test-imu-euler-x', data.euler[0].toFixed(2));
        updateElement('test-imu-euler-y', data.euler[1].toFixed(2));
        updateElement('test-imu-euler-z', data.euler[2].toFixed(2));
    }
    
    if (data.gyro && data.gyro.length === 3) {
        const gyroStr = data.gyro[0].toFixed(2) + ', ' + data.gyro[1].toFixed(2) + ', ' + data.gyro[2].toFixed(2);
        updateElement('gyro-value', gyroStr);
        
        // Update detailed gyro fields for each page
        updateElement('trackr-imu-gyro-x', data.gyro[0].toFixed(2));
        updateElement('trackr-imu-gyro-y', data.gyro[1].toFixed(2));
        updateElement('trackr-imu-gyro-z', data.gyro[2].toFixed(2));
        updateElement('teleop-imu-gyro-x', data.gyro[0].toFixed(2));
        updateElement('teleop-imu-gyro-y', data.gyro[1].toFixed(2));
        updateElement('teleop-imu-gyro-z', data.gyro[2].toFixed(2));
        updateElement('test-imu-gyro-x', data.gyro[0].toFixed(2));
        updateElement('test-imu-gyro-y', data.gyro[1].toFixed(2));
        updateElement('test-imu-gyro-z', data.gyro[2].toFixed(2));
    }
    
    if (data.accel && data.accel.length === 3) {
        const accelStr = data.accel[0].toFixed(2) + ', ' + data.accel[1].toFixed(2) + ', ' + data.accel[2].toFixed(2);
        updateElement('accel-value', accelStr);
        
        // Update detailed accel fields for each page
        updateElement('trackr-imu-accel-x', data.accel[0].toFixed(2));
        updateElement('trackr-imu-accel-y', data.accel[1].toFixed(2));
        updateElement('trackr-imu-accel-z', data.accel[2].toFixed(2));
        updateElement('teleop-imu-accel-x', data.accel[0].toFixed(2));
        updateElement('teleop-imu-accel-y', data.accel[1].toFixed(2));
        updateElement('teleop-imu-accel-z', data.accel[2].toFixed(2));
        updateElement('test-imu-accel-x', data.accel[0].toFixed(2));
        updateElement('test-imu-accel-y', data.accel[1].toFixed(2));
        updateElement('test-imu-accel-z', data.accel[2].toFixed(2));
    }
    
    // Update tracker-specific data
    if (data.pan_error !== undefined) {
        updateElement('pan-error', data.pan_error);
    }
    if (data.tilt_error !== undefined) {
        updateElement('tilt-error', data.tilt_error);
    }
    if (data.has_led !== undefined) {
        const ledStatus = document.getElementById('led-status');
        if (ledStatus) {
            ledStatus.textContent = data.has_led ? 'Yes' : 'No';
            ledStatus.className = data.has_led ? 'connected' : 'disconnected';
        }
        
        // Update lock status banner
        updateLockStatus(data.has_led, data.pan_error, data.tilt_error);
    }
}

function updateLockStatus(hasLed, panError, tiltError) {
    const lockStatus = document.getElementById('lock-status');
    const lockIcon = document.getElementById('lock-icon');
    const lockText = document.getElementById('lock-text');
    
    if (!lockStatus || !lockIcon || !lockText) return;
    
    if (hasLed) {
        // Check if we're locked on (errors are small)
        const errorThreshold = 50; // pixels
        const isLocked = Math.abs(panError) < errorThreshold && Math.abs(tiltError) < errorThreshold;
        
        if (isLocked) {
            lockStatus.className = 'lock-status locked';
            lockIcon.textContent = '';
            lockText.textContent = 'LOCKED ON TARGET';
        } else {
            lockStatus.className = 'lock-status tracking';
            lockIcon.textContent = '';
            lockText.textContent = 'TRACKING';
        }
    } else {
        lockStatus.className = 'lock-status searching';
        lockIcon.textContent = '';
        lockText.textContent = 'SEARCHING';
    }
}

function updateElement(id, value) {
    const element = document.getElementById(id);
    if (element) {
        element.textContent = value;
    }
}

function sendCommand(command) {
    if (ws && ws.readyState === WebSocket.OPEN) {
        ws.send(command);
        console.log('Sent:', command);
    } else {
        console.log('WebSocket not connected!');
    }
}

// Tracker Mode Functions
function updateThreshold(value) {
    document.getElementById('threshold-display').textContent = value;
    sendCommand('threshold:' + value);
}

function startTracking() {
    sendCommand('track:start');
    updateElement('trackr-status', 'Tracking Active');
}

function stopTracking() {
    sendCommand('track:stop');
    updateElement('trackr-status', 'Tracking Stopped');
}

function calibrateIMU() {
    sendCommand('calibrate');
    updateElement('trackr-status', 'Calibrating...');
}

// Teleop Mode Functions
function setControlMode(mode) {
    controlMode = mode;
    
    // Update button styles
    const velocityBtn = document.getElementById('velocity-mode-btn');
    const positionBtn = document.getElementById('position-mode-btn');
    
    if (mode === 'velocity') {
        velocityBtn.classList.add('active');
        positionBtn.classList.remove('active');
        
        // Show velocity controls, hide position controls
        document.getElementById('pan-velocity-controls').style.display = 'block';
        document.getElementById('pan-position-controls').style.display = 'none';
        document.getElementById('tilt-velocity-controls').style.display = 'block';
        document.getElementById('tilt-position-controls').style.display = 'none';
        
        // Update D-pad label
        document.getElementById('dpad-speed-label').innerHTML = 'Speed: <span id="dpad-speed-display">' + dpadSpeed + '</span>%';
    } else {
        velocityBtn.classList.remove('active');
        positionBtn.classList.add('active');
        
        // Show position controls, hide velocity controls
        document.getElementById('pan-velocity-controls').style.display = 'none';
        document.getElementById('pan-position-controls').style.display = 'block';
        document.getElementById('tilt-velocity-controls').style.display = 'none';
        document.getElementById('tilt-position-controls').style.display = 'block';
        
        // Update D-pad label
        document.getElementById('dpad-speed-label').innerHTML = 'Step: <span id="dpad-speed-display">' + dpadSpeed + '</span>°';
    }
}

function updateDpadSpeed(value) {
    dpadSpeed = parseInt(value);
    document.getElementById('dpad-speed-display').textContent = dpadSpeed;
}

function startDpadMove(direction) {
    // Prevent multiple intervals
    if (dpadInterval) return;
    
    // Send D-pad command to controller
    sendCommand('dpad:' + direction);
    
    // Continue sending while held
    dpadInterval = setInterval(() => sendCommand('dpad:' + direction), 100);
}

function executeDpadMove(direction) {
    if (controlMode === 'velocity') {
        // Velocity mode - send continuous velocity commands
        let panVel = 0;
        let tiltVel = 0;
        
        switch(direction) {
            case 'up':
                tiltVel = dpadSpeed;
                break;
            case 'down':
                tiltVel = -dpadSpeed;
                break;
            case 'left':
                panVel = -dpadSpeed;
                break;
            case 'right':
                panVel = dpadSpeed;
                break;
            case 'up-left':
                tiltVel = dpadSpeed;
                panVel = -dpadSpeed;
                break;
            case 'up-right':
                tiltVel = dpadSpeed;
                panVel = dpadSpeed;
                break;
            case 'down-left':
                tiltVel = -dpadSpeed;
                panVel = -dpadSpeed;
                break;
            case 'down-right':
                tiltVel = -dpadSpeed;
                panVel = dpadSpeed;
                break;
        }
        
        if (panVel !== 0) {
            sendCommand('pan:velocity:' + panVel);
        }
        if (tiltVel !== 0) {
            sendCommand('tilt:velocity:' + tiltVel);
        }
    } else {
        // Position mode - send incremental position changes
        const panPos = parseFloat(document.getElementById('teleop-pan-position').textContent) || 0;
        const tiltPos = parseFloat(document.getElementById('teleop-tilt-position').textContent) || 0;
        
        let newPanPos = panPos;
        let newTiltPos = tiltPos;
        
        switch(direction) {
            case 'up':
                newTiltPos = tiltPos + dpadSpeed;
                break;
            case 'down':
                newTiltPos = tiltPos - dpadSpeed;
                break;
            case 'left':
                newPanPos = panPos - dpadSpeed;
                break;
            case 'right':
                newPanPos = panPos + dpadSpeed;
                break;
            case 'up-left':
                newTiltPos = tiltPos + dpadSpeed;
                newPanPos = panPos - dpadSpeed;
                break;
            case 'up-right':
                newTiltPos = tiltPos + dpadSpeed;
                newPanPos = panPos + dpadSpeed;
                break;
            case 'down-left':
                newTiltPos = tiltPos - dpadSpeed;
                newPanPos = panPos - dpadSpeed;
                break;
            case 'down-right':
                newTiltPos = tiltPos - dpadSpeed;
                newPanPos = panPos + dpadSpeed;
                break;
        }
        
        sendCommand('pan:position:' + newPanPos);
        sendCommand('tilt:position:' + newTiltPos);
    }
}

function stopDpadMove() {
    if (dpadInterval) {
        clearInterval(dpadInterval);
        dpadInterval = null;
    }
    
    // Send stop command to D-pad shares
    sendCommand('dpad:stop');
}

function updatePanVelocity(value) {
    document.getElementById('pan-velocity-input').value = value;
}

function updateTiltVelocity(value) {
    document.getElementById('tilt-velocity-input').value = value;
}

function setPanVelocity() {
    const value = document.getElementById('pan-velocity-input').value;
    sendCommand('pan:velocity:' + value);
}

function setTiltVelocity() {
    const value = document.getElementById('tilt-velocity-input').value;
    sendCommand('tilt:velocity:' + value);
}

function setPanPosition() {
    const value = document.getElementById('pan-position-input').value;
    sendCommand('pan:position:' + value);
}

function setTiltPosition() {
    const value = document.getElementById('tilt-position-input').value;
    sendCommand('tilt:position:' + value);
}

function stopAll() {
    sendCommand('stop');
    document.getElementById('pan-velocity-slider').value = 0;
    document.getElementById('pan-velocity-input').value = 0;
    document.getElementById('tilt-velocity-slider').value = 0;
    document.getElementById('tilt-velocity-input').value = 0;
    updateElement('teleop-status', 'All Motors Stopped');
}

function zeroPanEncoder() {
    sendCommand('zero:pan');
    updateElement('test-status', 'Pan Encoder Zeroed');
}

function zeroTiltEncoder() {
    sendCommand('zero:tilt');
    updateElement('test-status', 'Tilt Encoder Zeroed');
}

function zeroAllEncoders() {
    sendCommand('zero');
    updateElement('test-status', 'All Encoders Zeroed');
}

function centerPosition() {
    sendCommand('center');
    updateElement('teleop-status', 'Centering...');
}

// Motor Test Mode Functions
function setTestControlMode(mode) {
    testControlMode = mode;
    
    // Update button styles
    const velocityBtn = document.getElementById('test-velocity-mode-btn');
    const positionBtn = document.getElementById('test-position-mode-btn');
    
    if (mode === 'velocity') {
        velocityBtn.classList.add('active');
        positionBtn.classList.remove('active');
        
        // Show velocity controls, hide position controls
        document.getElementById('test-pan-velocity-controls').style.display = 'block';
        document.getElementById('test-pan-position-controls').style.display = 'none';
        document.getElementById('test-tilt-velocity-controls').style.display = 'block';
        document.getElementById('test-tilt-position-controls').style.display = 'none';
    } else {
        velocityBtn.classList.remove('active');
        positionBtn.classList.add('active');
        
        // Show position controls, hide velocity controls
        document.getElementById('test-pan-velocity-controls').style.display = 'none';
        document.getElementById('test-pan-position-controls').style.display = 'block';
        document.getElementById('test-tilt-velocity-controls').style.display = 'none';
        document.getElementById('test-tilt-position-controls').style.display = 'block';
    }
}

function updateTestPanVelocity(value) {
    document.getElementById('test-pan-velocity-input').value = value;
}

function updateTestTiltVelocity(value) {
    document.getElementById('test-tilt-velocity-input').value = value;
}

function setTestPanVelocity() {
    const value = document.getElementById('test-pan-velocity-input').value;
    sendCommand('pan:velocity:' + value);
}

function setTestTiltVelocity() {
    const value = document.getElementById('test-tilt-velocity-input').value;
    sendCommand('tilt:velocity:' + value);
}

function setTestPanPosition() {
    const value = document.getElementById('test-pan-position-input').value;
    sendCommand('pan:position:' + value);
}

function setTestTiltPosition() {
    const value = document.getElementById('test-tilt-position-input').value;
    sendCommand('tilt:position:' + value);
}

// Calibration Mode Functions
function startIMUCalibration() {
    sendCommand('calibrate:imu:start');
    updateElement('imu-cal-status', 'Calibrating... Move sensor slowly');
    updateElement('cal-status', 'IMU Calibration in progress');
}

function loadIMUCalibration() {
    sendCommand('calibrate:imu:load');
    updateElement('imu-cal-status', 'Loading saved calibration');
    updateElement('cal-status', 'IMU Calibration loaded');
}

function setPanHome() {
    sendCommand('calibrate:pan:home');
    updateElement('cal-status', 'Pan home position set');
}

function setTiltZero() {
    sendCommand('calibrate:tilt:zero');
    updateElement('cal-status', 'Tilt zero position set');
}

function saveMotorCalibration() {
    sendCommand('calibrate:motor:save');
    updateElement('cal-status', 'Motor calibration saved');
}

// Connect when page loads
window.onload = function() {
    connectWebSocket();
    showPage('page-mode-select');
};