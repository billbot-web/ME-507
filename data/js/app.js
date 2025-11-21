// ESP32 Motor Control JavaScript

let ws = null;
let reconnectInterval = null;

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
    // Update motor data
    document.getElementById('velocity-value').textContent = (data.velocity || 0).toFixed(2);
    document.getElementById('position-value').textContent = (data.position || 0).toFixed(2);
    
    // Update IMU data
    if (data.euler && data.euler.length === 3) {
        document.getElementById('euler-value').textContent = 
            data.euler[0].toFixed(1) + ', ' + data.euler[1].toFixed(1) + ', ' + data.euler[2].toFixed(1);
    }
    
    if (data.gyro && data.gyro.length === 3) {
        document.getElementById('gyro-value').textContent = 
            data.gyro[0].toFixed(2) + ', ' + data.gyro[1].toFixed(2) + ', ' + data.gyro[2].toFixed(2);
    }
    
    if (data.accel && data.accel.length === 3) {
        document.getElementById('accel-value').textContent = 
            data.accel[0].toFixed(2) + ', ' + data.accel[1].toFixed(2) + ', ' + data.accel[2].toFixed(2);
    }
    
    // Update status
    document.getElementById('status-value').textContent = 'Last Update: ' + new Date().toLocaleTimeString();
}

function sendCommand(command) {
    if (ws && ws.readyState === WebSocket.OPEN) {
        ws.send(command);
        console.log('Sent:', command);
    } else {
        alert('WebSocket not connected!');
    }
}

function setVelocity() {
    const velocity = document.getElementById('velocity-input').value;
    if (velocity !== '') {
        sendCommand('velocity:' + velocity);
        document.getElementById('current-mode').textContent = 'Velocity Control';
    }
}

function setPosition() {
    const position = document.getElementById('position-input').value;
    if (position !== '') {
        sendCommand('position:' + position);
        document.getElementById('current-mode').textContent = 'Position Control';
    }
}

function stopMotor() {
    sendCommand('stop');
    document.getElementById('current-mode').textContent = 'Stopped';
}

function zeroEncoder() {
    sendCommand('zero');
    document.getElementById('current-mode').textContent = 'Zeroed';
}

// Connect when page loads
window.onload = function() {
    connectWebSocket();
};