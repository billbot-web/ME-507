#include <Arduino.h>
#include <WiFi.h>
#include <SPIFFS.h>
#include "UITask.h"
#include "IMU_Task.h"
#include "EncoderTask.h"  // for EncoderTask::Command enum

// Static instance
UITask* UITask::instance_ = nullptr;
String UITask::pendingMessage_ = "";
AsyncWebSocketClient* UITask::pendingClient_ = nullptr;

  UITask::UITask(Share<float>* tilt_positionShare,
               Share<float>*   tilt_velocityShare,
               Share<int8_t>* tilt_vref,
               Share<int16_t>* tilt_posref,
               Share<uint8_t>*   tilt_cmdShare,
               Share<bool>*     tilt_zeroShare,
               Share<float>* pan_positionShare,
               Share<float>*   pan_velocityShare,
               Share<int8_t>* pan_vref,
               Share<int16_t>* pan_posref,
               Share<uint8_t>*   pan_cmdShare,
               Share<bool>*     pan_zeroShare,
               Share<int8_t>*   dpad_pan,
               Share<int8_t>*   dpad_tilt,
               Share<bool>*     imu_mode,
               Share<uint8_t>*  ui_mode,
               Share<bool>*     motortest_mode,
               Share<bool>*     dcalibrate,
               Share<EulerAngles>* eulerAngles,
               Share<GyroData>* gyroData,
               Share<AccelData>* accelData,
               Share<bool>*     hasLed,
               Share<uint16_t>* ledThreshold,
               Share<int16_t>* pan_err,
               Share<int16_t>* tilt_err,
               uint32_t        updateMs) noexcept
  : tilt_positionShare_(tilt_positionShare),
    tilt_velocityShare_(tilt_velocityShare),
    tilt_vref_(tilt_vref),
    tilt_posref_(tilt_posref),
    tilt_cmdShare_(tilt_cmdShare),
    tilt_zeroShare_(tilt_zeroShare),
    pan_positionShare_(pan_positionShare),
    pan_velocityShare_(pan_velocityShare),
    pan_vref_(pan_vref),
    pan_posref_(pan_posref),
    pan_cmdShare_(pan_cmdShare),
    pan_zeroShare_(pan_zeroShare),
    dpad_pan_(dpad_pan),
    dpad_tilt_(dpad_tilt),
    imu_mode_(imu_mode),
    ui_mode_(ui_mode),
    motortest_mode_(motortest_mode),
    dcalibrate_(dcalibrate),
    eulerAngles_(eulerAngles),
    gyroData_(gyroData),
    accelData_(accelData),
    hasLed_(hasLed),
    ledThreshold_(ledThreshold),
    pan_err_(pan_err),
    tilt_err_(tilt_err),
    updateMs_(updateMs),
    fsm_(states_, 5)
{
  vref_ = tilt_vref;
  posref_ = tilt_posref;
  cmdShare_ = tilt_cmdShare;
  instance_ = this;
  ui_mode_->put(0);  // Default to mode 0 (choose mode)
  imu_mode_->put(false); // Default IMU mode to false
  dcalibrate_->put(false); // Default calibration to false
  motortest_mode_->put(false); // Default motor test mode to false
  tilt_cmdShare_->put(0);
  pan_cmdShare_->put(0);
  tilt_zeroShare_->put(false);
  pan_zeroShare_->put(false);
  
  // Create telemetry queues (length 2 = keep only most recent value)
  tilt_vel_queue_ = new Queue<float>(2, "TiltVelQ");
  tilt_pos_queue_ = new Queue<float>(2, "TiltPosQ");
  pan_vel_queue_ = new Queue<float>(2, "PanVelQ");
  pan_pos_queue_ = new Queue<float>(2, "PanPosQ");
  has_led_queue_ = new Queue<bool>(2, "HasLedQ");
}

// FreeRTOS C-style task entry function. Matches EncoderTask pattern where the
// task entry is provided with C linkage so it can be passed directly to
// xTaskCreate/xTaskCreatePinnedToCore.
extern "C" void ui_task_func(void* pvParameters) {
  UITask* UI_Task = static_cast<UITask*>(pvParameters);
  UITask::set_instance(UI_Task);
  const TickType_t tick = 250; // UITask default update period
  for (;;) {
    if (UI_Task) UI_Task->update();
    vTaskDelay(tick);
  }
}

// Main update method called by the FreeRTOS task
void UITask::update() noexcept
{
  // Read latest telemetry values from queues (non-blocking)
  // Encoder tasks push their latest values, we read if available
  // Queue::get() is blocking, so we just try to read and update cache
  
  // Read from queues - these will block briefly if empty but that's okay
  // since encoder tasks are constantly updating them
  if (tilt_vel_queue_ && tilt_vel_queue_->any()) {
    tilt_vel_queue_->get(cached_tilt_vel_);
  }
  if (tilt_pos_queue_ && tilt_pos_queue_->any()) {
    tilt_pos_queue_->get(cached_tilt_pos_);
  }
  if (pan_vel_queue_ && pan_vel_queue_->any()) {
    pan_vel_queue_->get(cached_pan_vel_);
  }
  if (pan_pos_queue_ && pan_pos_queue_->any()) {
    pan_pos_queue_->get(cached_pan_pos_);
  }
  if (has_led_queue_ && has_led_queue_->any()) {
    has_led_queue_->get(cached_has_led_);
  }
  
  // Read IMU data from shares (initialized at startup, so no blocking)
  if (eulerAngles_) {
    eulerAngles_->get(cached_euler_);
  }
  if (gyroData_) {
    gyroData_->get(cached_gyro_);
  }
  if (accelData_) {
    accelData_->get(cached_accel_);
  }
  
  // Read camera error data from shares
  if (pan_err_) {
    cached_pan_err_ = pan_err_->get();
  }
  if (tilt_err_) {
    cached_tilt_err_ = tilt_err_->get();
  }
  
  // Run the finite state machine
  fsm_.run_curstate();
  // Update web server (if initialized)
  updateWebServer();
}
// ---------------- States ----------------

// WAIT_FOR_INPUT: show prompt; parse Serial; forward commands
// ---------------- State implementations ----------------

/**
 * @brief CHOOSE_MODE state: Idle, waiting for mode selection via web interface
 * Transitions based on UI_mode and dcalibrate flags
 * @return Next state id
 */
uint8_t UITask::exec_choose_mode()
{
  if (!instance_) return -1;

  // Process any pending WebSocket messages
  if (pendingMessage_.length() > 0 && pendingClient_) {
    handleChooseModeMessage(pendingClient_, pendingMessage_);
    pendingMessage_ = "";
    pendingClient_ = nullptr;
  }

  // Check for calibration request
  if (instance_->dcalibrate_ && !instance_->dcalibrate_->get()) {
    Serial.println("[UITask] Transitioning to CALIBRATE state");
    return static_cast<int>(CALIBRATE);
  }

  // Check UI mode
  if (instance_->ui_mode_) {
    uint8_t mode = instance_->ui_mode_->get();
    if (mode == 1) {
      Serial.println("[UITask] Transitioning to TRACKER state");
      return static_cast<int>(TRACKER);
    } else if (mode == 2) {
      Serial.println("[UITask] Transitioning to TELEOP state");
      return static_cast<int>(TELEOP);
    } else if (mode == 3) {
      Serial.println("[UITask] Transitioning to MOTOR_TEST state");
      return static_cast<int>(MOTOR_TEST);
    }
  }

  // Stay in choose mode
  return static_cast<int>(CHOOSE_MODE);
}

/**
 * @brief CALIBRATE state: Handle calibration operations via web interface
 * @return Next state id
 */
uint8_t UITask::exec_calibrate()
{
  if (!instance_) return -1;

  // Process any pending WebSocket messages
  if (pendingMessage_.length() > 0 && pendingClient_) {
    handleCalibrateMessage(pendingClient_, pendingMessage_);
    pendingMessage_ = "";
    pendingClient_ = nullptr;
  }

  // Check if calibration is done
  if (instance_->dcalibrate_ && instance_->dcalibrate_->get()) {
    Serial.println("[UITask] Calibration complete, returning to CHOOSE_MODE");
    return static_cast<int>(CHOOSE_MODE);
  }

  // Continue calibration mode
  return static_cast<int>(CALIBRATE);
}

/**
 * @brief MOTOR_TEST state: Manual motor testing via web interface sliders
 * @return Next state id
 */
uint8_t UITask::exec_motor_test()
{
  if (!instance_) return -1;

  // Process any pending WebSocket messages
  if (pendingMessage_.length() > 0 && pendingClient_) {
    handleMotorTestMessage(pendingClient_, pendingMessage_);
    pendingMessage_ = "";
    pendingClient_ = nullptr;
  }

  // Check for mode change
  if (instance_->ui_mode_) {
    uint8_t mode = instance_->ui_mode_->get();
    if (mode == 0) {
      Serial.println("[UITask] Returning to CHOOSE_MODE");
      
      return static_cast<int>(CHOOSE_MODE);
    } else if (mode == 1) {
      Serial.println("[UITask] Switching to TRACKER state");
      
      return static_cast<int>(TRACKER);
    } else if (mode == 2) {
      Serial.println("[UITask] Switching to TELEOP state");
      
      return static_cast<int>(TELEOP);
    }
  }

  // Check for calibration request
  if (instance_->dcalibrate_ && !instance_->dcalibrate_->get()) {
    Serial.println("[UITask] Transitioning to CALIBRATE state");
    
    return static_cast<int>(CALIBRATE);
  }

  return static_cast<int>(MOTOR_TEST);
}

/**
 * @brief TELEOP state: Manual D-pad control mode
 * @return Next state id
 */
uint8_t UITask::exec_teleop()
{
  if (!instance_) return -1;

  // Process any pending WebSocket messages
  if (pendingMessage_.length() > 0 && pendingClient_) {
    handleTeleopMessage(pendingClient_, pendingMessage_);
    pendingMessage_ = "";
    pendingClient_ = nullptr;
  }

  // Check for mode change
  if (instance_->ui_mode_) {
    uint8_t mode = instance_->ui_mode_->get();
    if (mode == 0) {
      Serial.println("[UITask] Returning to CHOOSE_MODE");
      return static_cast<int>(CHOOSE_MODE);
    } else if (mode == 1) {
      Serial.println("[UITask] Switching to TRACKER state");
      return static_cast<int>(TRACKER);
    } else if (mode == 3) {
      Serial.println("[UITask] Switching to MOTOR_TEST state");
      return static_cast<int>(MOTOR_TEST);
    }
  }

  // Check for calibration request
  if (instance_->dcalibrate_ && !instance_->dcalibrate_->get()) {
    Serial.println("[UITask] Transitioning to CALIBRATE state");
    return static_cast<int>(CALIBRATE);
  }

  return static_cast<int>(TELEOP);
}

/**
 * @brief TRACKER state: Automatic tracking mode
 * @return Next state id
 */
uint8_t UITask::exec_tracker()
{
  if (!instance_) return -1;

  static bool printedState = false;
  if (!printedState) {
    Serial.println("[UITask] In TRACKER state");
    printedState = true;
  }

  // Process any pending WebSocket messages
  if (pendingMessage_.length() > 0 && pendingClient_) {
    handleTrackerMessage(pendingClient_, pendingMessage_);
    pendingMessage_ = "";
    pendingClient_ = nullptr;
  }

  // Check for mode change
  if (instance_->ui_mode_) {
    uint8_t mode = instance_->ui_mode_->get();
    if (mode == 0) {
      Serial.println("[UITask] Returning to CHOOSE_MODE");
      printedState = false;
      return static_cast<int>(CHOOSE_MODE);
    } else if (mode == 2) {
      Serial.println("[UITask] Switching to TELEOP state");
      printedState = false;
      return static_cast<int>(TELEOP);
    } else if (mode == 3) {
      Serial.println("[UITask] Switching to MOTOR_TEST state");
      printedState = false;
      return static_cast<int>(MOTOR_TEST);
    }
  }

  // Check for calibration request
  if (instance_->dcalibrate_ && !instance_->dcalibrate_->get()) {
    Serial.println("[UITask] Transitioning to CALIBRATE state");
    printedState = false;
    return static_cast<int>(CALIBRATE);
  }

  return static_cast<int>(TRACKER);
}

// ---------------- Web Server Methods ----------------

bool UITask::initWebServer(const String& ssid, const String& password)
{
  ssid_ = ssid;
  password_ = password;
  
  // Connect to WiFi first
  connectToWiFi();
  
  // Initialize SPIFFS
  if (!initSPIFFS()) {
    return false;
  }
  
  // Create AsyncWebServer instance
  server_ = new AsyncWebServer(80);
  ws_ = new AsyncWebSocket("/ws");
  
  // Set up WebSocket event handler
  ws_->onEvent(onWebSocketEvent);
  
  // Set up routes
  setupWebRoutes();
  
  // Start server
  server_->begin();
  Serial.println("Web server started on port 80");
  
  if (WiFi.getMode() == WIFI_AP) {
    Serial.print("Access the web interface at: http://");
    Serial.println(WiFi.softAPIP());
  } else if (WiFi.status() == WL_CONNECTED) {
    Serial.print("Access the web interface at: http://");
    Serial.println(WiFi.localIP());
  }
  
  return true;
}

void UITask::updateWebServer()
{
  // This will be called every update cycle, but only do something if web server is initialized
  if (server_ && (WiFi.status() == WL_CONNECTED || WiFi.getMode() == WIFI_AP)) {
    // Clean up disconnected clients periodically
    if (ws_) {
      ws_->cleanupClients();
    }
    
    // Broadcast telemetry periodically (but wait 5 seconds after startup to let tasks initialize)
    const unsigned long now = millis();
    if (now > 5000 && (now - lastTelemetryBroadcast_ >= telemetryInterval_)) {
      lastTelemetryBroadcast_ = now;
      broadcastTelemetry();
    }
  }
}

bool UITask::initSPIFFS()
{
  if (!SPIFFS.begin(true)) {
    Serial.println("SPIFFS Mount Failed");
    return false;
  }
  Serial.println("SPIFFS mounted successfully");
  return true;
}

void UITask::connectToWiFi()
{
  if (ssid_.length() == 0) {
    Serial.println("No WiFi credentials provided - running in Serial mode only");
    return;
  }
  
  // Set up ESP32 as an Access Point instead of connecting to existing network
  Serial.print("Creating WiFi Access Point: ");
  Serial.println(ssid_);
  
  // Configure Access Point
  WiFi.mode(WIFI_AP);
  bool apStarted = WiFi.softAP(ssid_.c_str(), password_.c_str());
  
  if (apStarted) {
    Serial.println("Access Point started successfully!");
    Serial.print("Network name (SSID): ");
    Serial.println(ssid_);
    Serial.print("Password: ");
    Serial.println(password_);
    Serial.print("IP address: ");
    Serial.println(WiFi.softAPIP());
    Serial.println("Connect your phone/laptop to this network, then go to: http://192.168.4.1");
  } else {
    Serial.println("Failed to start Access Point. Running in Serial mode only.");
  }
}

void UITask::setupWebRoutes()
{
  if (!server_) return;
  
  // Diagnostic test page at /test
  server_->on("/test", HTTP_GET, [](AsyncWebServerRequest *request){
    String html = "<!DOCTYPE html><html><head><title>ESP32 Motor Control Test</title>";
    html += "<style>body{font-family:Arial;max-width:800px;margin:20px auto;padding:20px;}";
    html += ".status{background:#e8f4fd;padding:15px;border-radius:5px;margin:10px 0;}";
    html += ".connected{color:green;} .disconnected{color:red;}</style></head><body>";
    html += "<h1>ESP32 Motor Control - WebSocket Test</h1>";
    html += "<div class='status'><p><strong>WebSocket:</strong> <span id='ws-status' class='disconnected'>Connecting...</span></p></div>";
    html += "<div id='output'></div>";
    html += "<button onclick='ws.send(\"stop\")'>Test Send</button>";
    html += "<script>";
    html += "let ws = new WebSocket('ws://' + location.hostname + '/ws');";
    html += "ws.onopen = () => { document.getElementById('ws-status').textContent = 'Connected'; document.getElementById('ws-status').className = 'connected'; };";
    html += "ws.onclose = () => { document.getElementById('ws-status').textContent = 'Disconnected'; document.getElementById('ws-status').className = 'disconnected'; };";
    html += "ws.onerror = (e) => { document.getElementById('output').innerHTML += 'Error: ' + e + '<br>'; };";
    html += "ws.onmessage = (e) => { document.getElementById('output').innerHTML += 'Received: ' + e.data + '<br>'; };";
    html += "</script></body></html>";
    request->send(200, "text/html", html);
  });
  
  // Serve static files from SPIFFS (normal UI)
  server_->serveStatic("/", SPIFFS, "/").setDefaultFile("index.html");

  

  
  // API endpoints
  setupAPI();
}

void UITask::setupAPI()
{
  if (!server_ || !ws_) return;
  
  // Add WebSocket handler
  server_->addHandler(ws_);
  
  // Simple test page if SPIFFS fails
  server_->on("/test", HTTP_GET, [](AsyncWebServerRequest *request){
    String html = "<!DOCTYPE html><html><head><title>ESP32 Motor Control - Test</title></head><body>";
    html += "<h1>ESP32 Motor Control - Test Page</h1>";
    html += "<p>If you see this, the web server is working!</p>";
    html += "<p>WebSocket connection test:</p>";
    html += "<button onclick='testWS()'>Test WebSocket</button>";
    html += "<div id='output'>Connecting...</div>";
    html += "<script>";
    html += "let ws = new WebSocket('ws://192.168.4.1/ws');";
    html += "ws.onopen = () => document.getElementById('output').innerHTML = 'WebSocket Connected!';";
    html += "ws.onclose = () => document.getElementById('output').innerHTML = 'WebSocket Disconnected';";
    html += "function testWS() { if(ws.readyState === 1) ws.send('stop'); }";
    html += "</script></body></html>";
    request->send(200, "text/html", html);
  });

  // REST API endpoints
  server_->on("/api/status", HTTP_GET, [](AsyncWebServerRequest *request){
    if (!instance_) {
      request->send(500, "application/json", "{\"error\":\"No UITask instance\"}");
      return;
    }

    float vel = 0.0f, pos = 0.0f;
    if (instance_->velocityShare_) vel = instance_->velocityShare_->get();
    if (instance_->positionShare_) pos = instance_->positionShare_->get();
    
    // Get IMU data
    EulerAngles euler = {0, 0, 0};
    GyroData gyro = {0, 0, 0};
    AccelData accel = {0, 0, 0};
    
    if (instance_->eulerAngles_) euler = instance_->eulerAngles_->get();
    if (instance_->gyroData_) gyro = instance_->gyroData_->get();
    if (instance_->accelData_) accel = instance_->accelData_->get();
    
    String json = "{";
    json += "\"velocity\":" + String(vel, 2) + ",";
    json += "\"position\":" + String(pos, 2) + ",";
    json += "\"euler\":[" + String(euler.x, 1) + "," + String(euler.y, 1) + "," + String(euler.z, 1) + "],";
    json += "\"gyro\":[" + String(gyro.x, 2) + "," + String(gyro.y, 2) + "," + String(gyro.z, 2) + "],";
    json += "\"accel\":[" + String(accel.x, 2) + "," + String(accel.y, 2) + "," + String(accel.z, 2) + "],";
    json += "\"wifi_ap_mode\":" + String(WiFi.getMode() == WIFI_AP ? "true" : "false") + ",";
    json += "\"wifi_connected\":" + String(WiFi.status() == WL_CONNECTED ? "true" : "false") + ",";
    json += "\"free_heap\":" + String(ESP.getFreeHeap());
    json += "}";
    
    request->send(200, "application/json", json);
  });
}

String UITask::createTelemetryMessage()
{
  // Use static buffer to reduce heap fragmentation
  static char buffer[512];
  
  // Use cached values (updated in update() loop) to avoid mutex blocking
  // This prevents deadlock when called from WebSocket async context
  float tilt_vel = cached_tilt_vel_;
  float tilt_pos = cached_tilt_pos_;
  float pan_vel = cached_pan_vel_;
  float pan_pos = cached_pan_pos_;
  bool has_led = cached_has_led_;
  
  // Build JSON using snprintf (more efficient than String concatenation)
  int len = snprintf(buffer, sizeof(buffer),
    "{\"type\":\"telemetry\",\"tilt_velocity\":%.2f,\"tilt_position\":%.2f,\"pan_velocity\":%.2f,\"pan_position\":%.2f,\"has_led\":%s,"
    "\"pan_error\":%d,\"tilt_error\":%d,"
    "\"euler\":[%.2f,%.2f,%.2f],"
    "\"gyro\":[%.2f,%.2f,%.2f],"
    "\"accel\":[%.2f,%.2f,%.2f]}",
    tilt_vel, tilt_pos, pan_vel, pan_pos, has_led ? "true" : "false",
    cached_pan_err_, cached_tilt_err_,
    cached_euler_.x, cached_euler_.y, cached_euler_.z,
    cached_gyro_.x, cached_gyro_.y, cached_gyro_.z,
    cached_accel_.x, cached_accel_.y, cached_accel_.z);
  
  return String(buffer);
}

void UITask::broadcastTelemetry()
{
  if (!ws_) {
    return;
  }
  
  // Safety check: only broadcast if we have clients
  size_t clientCount = ws_->count();
  if (clientCount == 0 || clientCount > 5) {
    return;
  }
  
  // Create message from cached values
  String message = createTelemetryMessage();
  
  // Validate message before sending
  if (message.length() == 0 || message.length() >= 512) {
    return;
  }
  
  // Broadcast to all clients
  ws_->textAll(message);
}

void UITask::processWebSocketMessage(AsyncWebSocketClient* client, const String& message)
{
  // Store message for processing in state function (different task context)
  pendingMessage_ = message;
  pendingClient_ = client;
}

// ---------------- State-Specific Message Handlers ----------------

void UITask::handleChooseModeMessage(AsyncWebSocketClient* client, const String& message)
{
  if (!instance_) return;

  Serial.println(message);
  
  // Mode selection commands
  if (message == "mode:idle") {
    Serial.println("[CHOOSE_MODE] Mode: Idle (staying in choose mode)");
    if (instance_->ui_mode_) instance_->ui_mode_->put(0); // 0 = CHOOSE_MODE
    if(instance_->tilt_cmdShare_) instance_->tilt_cmdShare_->put(0); // wait
    if(instance_->pan_cmdShare_) instance_->pan_cmdShare_->put(0); // wait
    if(instance_->tilt_vref_) instance_->tilt_vref_->put(0);
    if(instance_->pan_vref_) instance_->pan_vref_->put(0);
    sendWebSocketResponse(client, "{\"status\":\"mode_idle\"}");
  }
  else if (message == "mode:trackr") {
    Serial.println("[CHOOSE_MODE] Mode: Tracker - setting ui_mode to 1");
    if (instance_->ui_mode_) instance_->ui_mode_->put(1); // 1 = TRACKER
    if (instance_->imu_mode_) instance_->imu_mode_->put(true);
    if(instance_->tilt_cmdShare_) instance_->tilt_cmdShare_->put(0); // wait
    if(instance_->pan_cmdShare_) instance_->pan_cmdShare_->put(0); // wait
    if(instance_->tilt_vref_) instance_->tilt_vref_->put(0);
    if(instance_->pan_vref_) instance_->pan_vref_->put(0);
    sendWebSocketResponse(client, "{\"status\":\"mode_trackr\"}");
  }
  else if (message == "mode:teleop") {
    Serial.println("[CHOOSE_MODE] Mode: Teleop - setting ui_mode to 2");
    if (instance_->ui_mode_) instance_->ui_mode_->put(2); // 2 = TELEOP
    if (instance_->imu_mode_) instance_->imu_mode_->put(true);
    if(instance_->tilt_cmdShare_) instance_->tilt_cmdShare_->put(1); // Velocity control
    if(instance_->pan_cmdShare_) instance_->pan_cmdShare_->put(1); // Velocity control
    if(instance_->tilt_vref_) instance_->tilt_vref_->put(0);
    if(instance_->pan_vref_) instance_->pan_vref_->put(0);
    sendWebSocketResponse(client, "{\"status\":\"mode_teleop\"}");
  }
  else if (message == "mode:test") {
    Serial.println("[CHOOSE_MODE] Mode: Motor Test - setting ui_mode to 3");
    if (instance_->ui_mode_) instance_->ui_mode_->put(3); // 3 = MOTOR_TEST
    if (instance_->imu_mode_) instance_->imu_mode_->put(true);
    sendWebSocketResponse(client, "{\"status\":\"mode_test\"}");
  }
  else if (message == "mode:calibrate") {
    Serial.println("[CHOOSE_MODE] Mode: Calibrate - setting dcalibrate flag");
    if (instance_->ui_mode_) instance_->ui_mode_->put(0); // Ensure ui_mode is 0
    if (instance_->imu_mode_) instance_->imu_mode_->put(false);
    if (instance_->motortest_mode_) instance_->motortest_mode_->put(false);
    sendWebSocketResponse(client, "{\"status\":\"mode_calibrate\"}");
  
  }
  // Basic control commands (available in choose mode)
  else if (message == "stop") {
    if (instance_->tilt_cmdShare_) instance_->tilt_cmdShare_->put(0);
    if (instance_->pan_cmdShare_) instance_->pan_cmdShare_->put(0);
    if (instance_->tilt_vref_) instance_->tilt_vref_->put(0);
    if (instance_->pan_vref_) instance_->pan_vref_->put(0);
    sendWebSocketResponse(client, "{\"status\":\"stopped\"}");
  }
}

void UITask::sendWebSocketResponse(AsyncWebSocketClient* client, const String& response)
{
  if (client && client->status() == WS_CONNECTED) {
    client->text(response);
  }
}

// Static callback for WebSocket events
void UITask::onWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, 
                             AwsEventType type, void *arg, uint8_t *data, size_t len)
{
  if (!instance_) return;
  
  switch (type) {
    case WS_EVT_CONNECT:
      Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
      break;
      
    case WS_EVT_DISCONNECT:
      Serial.printf("WebSocket client #%u disconnected\n", client->id());
      break;
      
    case WS_EVT_DATA: {
      AwsFrameInfo *info = (AwsFrameInfo*)arg;
      if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
        data[len] = 0; // null terminate
        String message = (char*)data;
        instance_->processWebSocketMessage(client, message);
      }
      break;
    }
      
    case WS_EVT_PONG:
    case WS_EVT_ERROR:
      break;
  }
}

uint32_t UITask::getWebSocketClientCount() const
{
  return ws_ ? ws_->count() : 0;
}

void UITask::handleCalibrateMessage(AsyncWebSocketClient* client, const String& message)
{
  if (!instance_) return;
  
  // Check for return to choose mode
  if (message == "mode:idle") {
    Serial.println("[CALIBRATE] Returning to CHOOSE_MODE");
    if (instance_->ui_mode_) instance_->ui_mode_->put(0);
    if (instance_->dcalibrate_) instance_->dcalibrate_->put(true);
    if (instance_->imu_mode_) instance_->imu_mode_->put(false);
    if(instance_->tilt_cmdShare_) instance_->tilt_cmdShare_->put(0); // wait
    if(instance_->pan_cmdShare_) instance_->pan_cmdShare_->put(0); // wait
    if(instance_->tilt_vref_) instance_->tilt_vref_->put(0);
    if(instance_->pan_vref_) instance_->pan_vref_->put(0);
    sendWebSocketResponse(client, "{\"status\":\"mode_idle\"}");
    return;
  }
  else if (message == "mode:trackr") {
    Serial.println("[CHOOSE_MODE] Mode: Tracker - setting ui_mode to 1");
    if (instance_->ui_mode_) instance_->ui_mode_->put(1); // 1 = TRACKER
    if (instance_->dcalibrate_) instance_->dcalibrate_->put(true);
    if (instance_->imu_mode_) instance_->imu_mode_->put(true);
    sendWebSocketResponse(client, "{\"status\":\"mode_trackr\"}");
  }
  else if (message == "mode:teleop") {
    Serial.println("[CHOOSE_MODE] Mode: Teleop - setting ui_mode to 2");
    if (instance_->ui_mode_) instance_->ui_mode_->put(2); // 2 = TELEOP
    if (instance_->imu_mode_) instance_->imu_mode_->put(true);
    if (instance_->dcalibrate_) instance_->dcalibrate_->put(true);
    if(instance_->tilt_cmdShare_) instance_->tilt_cmdShare_->put(1); // Velocity control
    if(instance_->pan_cmdShare_) instance_->pan_cmdShare_->put(1); // Velocity control
    if(instance_->tilt_vref_) instance_->tilt_vref_->put(0);
    if(instance_->pan_vref_) instance_->pan_vref_->put(0);
    sendWebSocketResponse(client, "{\"status\":\"mode_teleop\"}");
  }
  else if (message == "mode:test") {
    Serial.println("[CHOOSE_MODE] Mode: Motor Test - setting ui_mode to 3");
    if (instance_->ui_mode_) instance_->ui_mode_->put(3); // 3 = MOTOR_TEST
    if (instance_->imu_mode_) instance_->imu_mode_->put(true);
    if (instance_->dcalibrate_) instance_->dcalibrate_->put(true);
    sendWebSocketResponse(client, "{\"status\":\"mode_test\"}");
  }
  // Calibration-specific commands
  if (message.startsWith("calibrate:")) {
    String calibCmd = message.substring(10); // Remove "calibrate:" prefix
    
    if (calibCmd == "imu:start") {
      Serial.println("[CALIBRATE] Starting IMU calibration");
      // TODO: Trigger IMU calibration routine
      sendWebSocketResponse(client, "{\"status\":\"imu_calibration_started\"}");
    }
    else if (calibCmd == "imu:load") {
      Serial.println("[CALIBRATE] Loading IMU calibration data");
      // TODO: Load IMU calibration from storage
      sendWebSocketResponse(client, "{\"status\":\"imu_calibration_loaded\"}");
    }
    else if (calibCmd == "pan:home") {
      Serial.println("[CALIBRATE] Setting pan home position");
      if (instance_->pan_zeroShare_) instance_->pan_zeroShare_->put(true);
      sendWebSocketResponse(client, "{\"status\":\"pan_home_set\"}");
    }
    else if (calibCmd == "tilt:zero") {
      Serial.println("[CALIBRATE] Setting tilt zero position");
      if (instance_->tilt_zeroShare_) instance_->tilt_zeroShare_->put(true);
      // TODO: Calculate and store tilt limits from this zero position
      sendWebSocketResponse(client, "{\"status\":\"tilt_zero_set\"}");
    }
    else if (calibCmd == "motor:save") {
      Serial.println("[CALIBRATE] Saving motor calibration data");
      if(instance_->dcalibrate_) instance_->dcalibrate_->put(true);
      Serial.println("[CALIBRATE] Motor calibration saved (placeholder)");
      sendWebSocketResponse(client, "{\"status\":\"calibration_saved\"}");
    }
    else if (calibCmd == "done") {
      Serial.println("[CALIBRATE] Exiting calibration mode");
      if (instance_->dcalibrate_) instance_->dcalibrate_->put(true);
      sendWebSocketResponse(client, "{\"status\":\"calibration_done\"}");
    }
  }
  // Allow stop command in calibrate
  else if (message == "stop") {
    if (instance_->tilt_cmdShare_) instance_->tilt_cmdShare_->put(static_cast<uint8_t>(CHOOSE_MODE));
    if (instance_->pan_cmdShare_) instance_->pan_cmdShare_->put(static_cast<uint8_t>(CHOOSE_MODE));
    if (instance_->tilt_vref_) instance_->tilt_vref_->put(0);
    if (instance_->pan_vref_) instance_->pan_vref_->put(0);
    sendWebSocketResponse(client, "{\"status\":\"stopped\"}");
  }
}

void UITask::handleMotorTestMessage(AsyncWebSocketClient* client, const String& message)
{
  if (!instance_) return;
  
  // Check for return to choose mode
  if (message == "mode:idle") {
    Serial.println("[MOTOR_TEST] Returning to CHOOSE_MODE");
    if (instance_->ui_mode_) instance_->ui_mode_->put(0);
    if (instance_->imu_mode_) instance_->imu_mode_->put(false);
    if(instance_->tilt_cmdShare_) instance_->tilt_cmdShare_->put(0); // wait
    if(instance_->pan_cmdShare_) instance_->pan_cmdShare_->put(0); // wait
    if(instance_->tilt_vref_) instance_->tilt_vref_->put(0);
    if(instance_->pan_vref_) instance_->pan_vref_->put(0);
    sendWebSocketResponse(client, "{\"status\":\"mode_idle\"}");
    return;
  }
  else if (message == "mode:trackr") {
    Serial.println("[CHOOSE_MODE] Mode: Tracker - setting ui_mode to 1");
    if (instance_->ui_mode_) instance_->ui_mode_->put(1); // 1 = TRACKER
    if (instance_->imu_mode_) instance_->imu_mode_->put(true);
    sendWebSocketResponse(client, "{\"status\":\"mode_trackr\"}");
  }
  else if (message == "mode:teleop") {
    Serial.println("[CHOOSE_MODE] Mode: Teleop - setting ui_mode to 2");
    if (instance_->ui_mode_) instance_->ui_mode_->put(2); // 2 = TELEOP
    if (instance_->imu_mode_) instance_->imu_mode_->put(true);
    if(instance_->tilt_cmdShare_) instance_->tilt_cmdShare_->put(1); // Velocity control
    if(instance_->pan_cmdShare_) instance_->pan_cmdShare_->put(1); // Velocity control
    if(instance_->tilt_vref_) instance_->tilt_vref_->put(0);
    if(instance_->pan_vref_) instance_->pan_vref_->put(0);
    sendWebSocketResponse(client, "{\"status\":\"mode_teleop\"}");
  }
  else if (message == "mode:calibrate") {
    Serial.println("[CHOOSE_MODE] Mode: Calibrate - setting dcalibrate flag");
    if (instance_->dcalibrate_) instance_->dcalibrate_->put(false); // Use dcalibrate flag
    if (instance_->ui_mode_) instance_->ui_mode_->put(0); // Ensure ui_mode is 0
    if (instance_->imu_mode_) instance_->imu_mode_->put(false);
    if (instance_->motortest_mode_) instance_->motortest_mode_->put(false);
    sendWebSocketResponse(client, "{\"status\":\"mode_calibrate\"}");
  }
  // Motor test commands - full velocity and position control
  if (message.startsWith("pan:velocity:")) {
    float vel = message.substring(13).toFloat();
    Serial.println("[MOTOR_TEST] Setting pan velocity to " + String(vel));
    if (instance_->pan_vref_) instance_->pan_vref_->put(static_cast<int8_t>(vel));
    if (instance_->pan_cmdShare_) instance_->pan_cmdShare_->put(static_cast<uint8_t>(1));
    sendWebSocketResponse(client, "{\"status\":\"pan_velocity_set\",\"value\":" + String(vel) + "}");
  }
  else if (message.startsWith("pan:position:")) {
    float pos = message.substring(13).toFloat();
    if (instance_->pan_posref_) instance_->pan_posref_->put(static_cast<int16_t>(pos));
    if (instance_->pan_cmdShare_) instance_->pan_cmdShare_->put(static_cast<uint8_t>(2));
    sendWebSocketResponse(client, "{\"status\":\"pan_position_set\",\"value\":" + String(pos) + "}");
  }
  else if (message.startsWith("tilt:velocity:")) {
    float vel = message.substring(14).toFloat();
    Serial.println("[MOTOR_TEST] Setting tilt velocity to " + String(vel));
    if (instance_->tilt_vref_) instance_->tilt_vref_->put(static_cast<int8_t>(vel));
    if (instance_->tilt_cmdShare_) instance_->tilt_cmdShare_->put(static_cast<uint8_t>(1));
    sendWebSocketResponse(client, "{\"status\":\"tilt_velocity_set\",\"value\":" + String(vel) + "}");
  }
  else if (message.startsWith("tilt:position:")) {
    float pos = message.substring(14).toFloat();
    if (instance_->tilt_posref_) instance_->tilt_posref_->put(static_cast<int16_t>(pos));
    if (instance_->tilt_cmdShare_) instance_->tilt_cmdShare_->put(static_cast<uint8_t>(2));
    sendWebSocketResponse(client, "{\"status\":\"tilt_position_set\",\"value\":" + String(pos) + "}");
  }
  else if (message == "stop") {
    if (instance_->tilt_cmdShare_) instance_->tilt_cmdShare_->put(static_cast<uint8_t>(StateId::CHOOSE_MODE));
    if (instance_->pan_cmdShare_) instance_->pan_cmdShare_->put(static_cast<uint8_t>(StateId::CHOOSE_MODE));
    if (instance_->tilt_vref_) instance_->tilt_vref_->put(0);
    if (instance_->pan_vref_) instance_->pan_vref_->put(0);
    instance_->sendWebSocketResponse(client, "{\"status\":\"stopped\"}");
  }
  else if (message == "center") {
    // Center position (go to 0,0)
    if (instance_->tilt_posref_) instance_->tilt_posref_->put(0);
    if (instance_->pan_posref_) instance_->pan_posref_->put(0);
    if (instance_->tilt_cmdShare_) instance_->tilt_cmdShare_->put(static_cast<uint8_t>(2));
    if (instance_->pan_cmdShare_) instance_->pan_cmdShare_->put(static_cast<uint8_t>(2));
    instance_->sendWebSocketResponse(client, "{\"status\":\"centering\"}");
  }
}

void UITask::handleTeleopMessage(AsyncWebSocketClient* client, const String& message)
{
  if (!instance_) return;
  
  // Check for return to choose mode
  if (message == "mode:idle") {
    Serial.println("[TELEOP] Returning to CHOOSE_MODE");
    if (instance_->ui_mode_) instance_->ui_mode_->put(0);
    if (instance_->imu_mode_) instance_->imu_mode_->put(false);
    // Stop D-pad
    if (instance_->dpad_pan_) instance_->dpad_pan_->put(0);
    if (instance_->dpad_tilt_) instance_->dpad_tilt_->put(0);
    if(instance_->tilt_cmdShare_) instance_->tilt_cmdShare_->put(0); // wait
    if(instance_->pan_cmdShare_) instance_->pan_cmdShare_->put(0); // wait
    if(instance_->tilt_vref_) instance_->tilt_vref_->put(0);
    if(instance_->pan_vref_) instance_->pan_vref_->put(0);
    sendWebSocketResponse(client, "{\"status\":\"mode_idle\"}");
    return;
  }
  else if (message == "mode:trackr") {
    Serial.println("[CHOOSE_MODE] Mode: Tracker - setting ui_mode to 1");
    if (instance_->ui_mode_) instance_->ui_mode_->put(1); // 1 = TRACKER
    if (instance_->imu_mode_) instance_->imu_mode_->put(true);
    sendWebSocketResponse(client, "{\"status\":\"mode_trackr\"}");
  }
  else if (message == "mode:test") {
    Serial.println("[CHOOSE_MODE] Mode: Motor Test - setting ui_mode to 3");
    if (instance_->ui_mode_) instance_->ui_mode_->put(3); // 3 = MOTOR_TEST
    if (instance_->imu_mode_) instance_->imu_mode_->put(true);

    sendWebSocketResponse(client, "{\"status\":\"mode_test\"}");
  }
  else if (message == "mode:calibrate") {
    Serial.println("[CHOOSE_MODE] Mode: Calibrate - setting dcalibrate flag");
    if (instance_->dcalibrate_) instance_->dcalibrate_->put(false); // Use dcalibrate flag
    if (instance_->ui_mode_) instance_->ui_mode_->put(0); // Ensure ui_mode is 0
    if (instance_->imu_mode_) instance_->imu_mode_->put(false);
    if (instance_->motortest_mode_) instance_->motortest_mode_->put(false);
    sendWebSocketResponse(client, "{\"status\":\"mode_calibrate\"}");
  }
  Serial.print(message);
  //enable velocity control on dpad command
  if (message.startsWith("dpad:")) {
    if (instance_->tilt_cmdShare_) instance_->tilt_cmdShare_->put(1);
    if (instance_->pan_cmdShare_) instance_->pan_cmdShare_->put(1);
  }
  if (message == "dpad:up") {
    if (instance_->dpad_tilt_) instance_->dpad_tilt_->put(1);
    if (instance_->dpad_pan_) instance_->dpad_pan_->put(0);
    sendWebSocketResponse(client, "{\"status\":\"dpad_up\"}");
  }
  else if (message == "dpad:down") {
    if (instance_->dpad_tilt_) instance_->dpad_tilt_->put(-1);
    if (instance_->dpad_pan_) instance_->dpad_pan_->put(0);
    sendWebSocketResponse(client, "{\"status\":\"dpad_down\"}");
  }
  else if (message == "dpad:left") {
    if (instance_->dpad_pan_) instance_->dpad_pan_->put(-1);
    if (instance_->dpad_tilt_) instance_->dpad_tilt_->put(0);
    sendWebSocketResponse(client, "{\"status\":\"dpad_left\"}");
  }
  else if (message == "dpad:right") {
    if (instance_->dpad_pan_) instance_->dpad_pan_->put(1);
    if (instance_->dpad_tilt_) instance_->dpad_tilt_->put(0);
    sendWebSocketResponse(client, "{\"status\":\"dpad_right\"}");
  }
  else if (message == "dpad:up-left") {
    if (instance_->dpad_tilt_) instance_->dpad_tilt_->put(1);
    if (instance_->dpad_pan_) instance_->dpad_pan_->put(-1);
    sendWebSocketResponse(client, "{\"status\":\"dpad_up_left\"}");
  }
  else if (message == "dpad:up-right") {
    if (instance_->dpad_tilt_) instance_->dpad_tilt_->put(1);
    if (instance_->dpad_pan_) instance_->dpad_pan_->put(1);
    sendWebSocketResponse(client, "{\"status\":\"dpad_up_right\"}");
  }
  else if (message == "dpad:down-left") {
    if (instance_->dpad_tilt_) instance_->dpad_tilt_->put(-1);
    if (instance_->dpad_pan_) instance_->dpad_pan_->put(-1);
    sendWebSocketResponse(client, "{\"status\":\"dpad_down_left\"}");
  }
  else if (message == "dpad:down-right") {
    if (instance_->dpad_tilt_) instance_->dpad_tilt_->put(-1);
    if (instance_->dpad_pan_) instance_->dpad_pan_->put(1);
    sendWebSocketResponse(client, "{\"status\":\"dpad_down_right\"}");
  }
  else if (message == "dpad:center" || message == "dpad:stop") {
    if (instance_->dpad_pan_) instance_->dpad_pan_->put(0);
    if (instance_->dpad_tilt_) instance_->dpad_tilt_->put(0);
    sendWebSocketResponse(client, "{\"status\":\"dpad_stop\"}");
  }
  else if (message == "stop") {
    if (instance_->tilt_cmdShare_) instance_->tilt_cmdShare_->put(0);
    if (instance_->pan_cmdShare_) instance_->pan_cmdShare_->put(0);
    if (instance_->tilt_vref_) instance_->tilt_vref_->put(0);
    if (instance_->pan_vref_) instance_->pan_vref_->put(0);
    if (instance_->dpad_pan_) instance_->dpad_pan_->put(0);
    if (instance_->dpad_tilt_) instance_->dpad_tilt_->put(0);
    sendWebSocketResponse(client, "{\"status\":\"stopped\"}");
  }
}

void UITask::handleTrackerMessage(AsyncWebSocketClient* client, const String& message)
{
  if (!instance_) return;
  
  // Check for return to choose mode
  if (message == "mode:idle") {
    Serial.println("[TRACKER] Returning to CHOOSE_MODE");
    if (instance_->ui_mode_) instance_->ui_mode_->put(0);
    if (instance_->imu_mode_) instance_->imu_mode_->put(false);
    if(instance_->tilt_cmdShare_) instance_->tilt_cmdShare_->put(0); // wait
    if(instance_->pan_cmdShare_) instance_->pan_cmdShare_->put(0); // wait
    if(instance_->tilt_vref_) instance_->tilt_vref_->put(0);
    if(instance_->pan_vref_) instance_->pan_vref_->put(0);
    sendWebSocketResponse(client, "{\"status\":\"mode_idle\"}");
    return;
  }
  else if (message == "mode:teleop") {
    Serial.println("[CHOOSE_MODE] Mode: Teleop - setting ui_mode to 2");
    if (instance_->ui_mode_) instance_->ui_mode_->put(2); // 2 = TELEOP
    if (instance_->imu_mode_) instance_->imu_mode_->put(true);
    if(instance_->tilt_cmdShare_) instance_->tilt_cmdShare_->put(1); // Velocity control
    if(instance_->pan_cmdShare_) instance_->pan_cmdShare_->put(1); // Velocity control
    if(instance_->tilt_vref_) instance_->tilt_vref_->put(0);
    if(instance_->pan_vref_) instance_->pan_vref_->put(0);
    sendWebSocketResponse(client, "{\"status\":\"mode_teleop\"}");
  }
  else if (message == "mode:test") {
    Serial.println("[CHOOSE_MODE] Mode: Motor Test - setting ui_mode to 3");
    if (instance_->ui_mode_) instance_->ui_mode_->put(3); // 3 = MOTOR_TEST
    if (instance_->imu_mode_) instance_->imu_mode_->put(true);
    if(instance_->tilt_cmdShare_) instance_->tilt_cmdShare_->put(1); // Velocity control
    if(instance_->pan_cmdShare_) instance_->pan_cmdShare_->put(1); // Velocity control
    if(instance_->tilt_vref_) instance_->tilt_vref_->put(0);
    if(instance_->pan_vref_) instance_->pan_vref_->put(0);
    sendWebSocketResponse(client, "{\"status\":\"mode_test\"}");
  }
  else if (message == "mode:calibrate") {
    Serial.println("[CHOOSE_MODE] Mode: Calibrate - setting dcalibrate flag");
    if (instance_->dcalibrate_) instance_->dcalibrate_->put(false); // Use dcalibrate flag
    if (instance_->ui_mode_) instance_->ui_mode_->put(0); // Ensure ui_mode is 0
    if (instance_->imu_mode_) instance_->imu_mode_->put(false);
    if (instance_->motortest_mode_) instance_->motortest_mode_->put(false);
    sendWebSocketResponse(client, "{\"status\":\"mode_calibrate\"}");
  }
  
  // Tracker mode commands
  if (message.startsWith("threshold:")) {
    int threshold = message.substring(10).toInt();
    Serial.printf("[TRACKER] LED threshold set to: %d\n", threshold);
    if (instance_->ledThreshold_) instance_->ledThreshold_->put(static_cast<uint16_t>(threshold));
    sendWebSocketResponse(client, "{\"status\":\"threshold_set\",\"value\":" + String(threshold) + "}");
  }
  else if (message == "track:start") {
    Serial.println("[TRACKER] Tracking started");
    if(instance_-> ui_mode_) instance_->ui_mode_->put(1);
    sendWebSocketResponse(client, "{\"status\":\"tracking_started\"}");
  }
  else if (message == "track:stop") {
    Serial.println("[TRACKER] Tracking stopped");
    if (instance_-> ui_mode_) instance_->ui_mode_->put(0);
    if (instance_->tilt_vref_) instance_->tilt_vref_->put(0);
    if (instance_->pan_vref_) instance_->pan_vref_->put(0);
    sendWebSocketResponse(client, "{\"status\":\"tracking_stopped\"}");
  }
  else if (message == "stop") {
    if (instance_-> ui_mode_) instance_->ui_mode_->put(0);
    if (instance_->tilt_vref_) instance_->tilt_vref_->put(0);
    if (instance_->pan_vref_) instance_->pan_vref_->put(0);
    sendWebSocketResponse(client, "{\"status\":\"stopped\"}");
  }
}
