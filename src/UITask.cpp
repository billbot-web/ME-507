#include <Arduino.h>
#include <WiFi.h>
#include <SPIFFS.h>
#include "UITask.h"
#include "EncoderTask.h"  // for EncoderTask::Command enum

// Static instance
UITask* UITask::instance_ = nullptr;

  UITask::UITask(Share<float>* positionShare,
               Share<float>*   velocityShare,
               Share<int8_t>* vref,
               Share<int16_t>* posref,
               Share<int8_t>*   cmdShare,
               Share<EulerAngles>* eulerAngles,
               Share<GyroData>* gyroData,
               Share<AccelData>* accelData,
               uint32_t        updateMs) noexcept
  : positionShare_(positionShare),
    velocityShare_(velocityShare),
    vref_(vref),
    posref_(posref),
    cmdShare_(cmdShare),
    eulerAngles_(eulerAngles),
    gyroData_(gyroData),
    accelData_(accelData),
    updateMs_(updateMs),
    fsm_(states_, 3)
{
  instance_ = this;
}

// FreeRTOS C-style task entry function. Matches EncoderTask pattern where the
// task entry is provided with C linkage so it can be passed directly to
// xTaskCreate/xTaskCreatePinnedToCore.
extern "C" void ui_task_func(void* pvParameters) {
  UITask* UI_Task = static_cast<UITask*>(pvParameters);
  UITask::set_instance(UI_Task);
  const TickType_t tick = pdMS_TO_TICKS(UI_Task ? UI_Task->get_updateMs() : 200);
  for (;;) {
    if (UI_Task) UI_Task->update();
    vTaskDelay(tick);
  }
}

// Main update method called by the FreeRTOS task
void UITask::update() noexcept
{
  // Run the finite state machine
  fsm_.run_curstate();
  // Update web server (if initialized)
  updateWebServer();
}

// ---------------- Helpers ----------------

void UITask::printHelpOnce()
{
  if (!instance_) return;
  const uint32_t now = millis();
  if (now - instance_->lastMenuPrintMs_ < 1000) return; // throttle menu spam
  instance_->lastMenuPrintMs_ = now;
  Serial.println();
  Serial.println("UI commands: v=velocity, s=stop, z=zero, h/?=help");
}

// Send encoder commands if queue is available
void UITask::sendEncoderCmdZero()
{
  if (!instance_ || !instance_->cmdShare_) return;
  const int8_t code = static_cast<int8_t>(3); // ZERO command is now 3
  instance_->cmdShare_->put(code);
}

void UITask::sendEncoderCmdStart()
{
  if (!instance_ || !instance_->cmdShare_) return;
  const int8_t code = static_cast<int8_t>(UITask::Command::VELOCITY_RUN);
  instance_->cmdShare_->put(code);
}

void UITask::sendEncoderCmdStop()
{
  if (!instance_ || !instance_->cmdShare_) return;
  const int8_t code = static_cast<int8_t>(UITask::Command::STOP);
  Serial.println("[UITask] *** SENDING STOP COMMAND ***");
  instance_->cmdShare_->put(code);
}

// ---------------- States ----------------

// WAIT_FOR_INPUT: show prompt; parse serial; forward commands
uint8_t UITask::exec_waitForInput()
{
  if (!instance_) return -1;

  // Print the appropriate prompt based on input mode
  if (!instance_->menuPrinted_) {
    Serial.println();
    if (instance_->inputMode_ == 0) {
      Serial.print("Select mode - v for velocity, p for position, z to zero: ");
    } else if (instance_->inputMode_ == 'v') {
      Serial.print("Enter velocity (-100 to 100): ");
    } else if (instance_->inputMode_ == 'p') {
      Serial.print("Enter position: ");
    }
    instance_->menuPrinted_ = true;
    instance_->inputPos_ = 0;
    instance_->inputBuf_[0] = '\0';
  }

  while (Serial.available() > 0) {
    char ch = static_cast<char>(Serial.read());
    // Echo back
    Serial.print(ch);
    
    if (ch == '\r' || ch == '\n') {
      if (instance_->inputPos_ == 0) {
        // empty line, ignore
        continue;
      }
      instance_->inputBuf_[instance_->inputPos_] = '\0';
      
      if (instance_->inputMode_ == 0) {
        // Mode selection step
        if (instance_->inputPos_ == 1) {
          char mode = instance_->inputBuf_[0];
          if (mode == 'z' || mode == 'Z') {
            // Zero command - execute immediately
            sendEncoderCmdZero();
            Serial.println();
            Serial.print("Select mode - v for velocity, p for position, z to zero: ");
            instance_->inputPos_ = 0;
            instance_->inputBuf_[0] = '\0';
            continue;
          } else if (mode == 'v' || mode == 'V') {
            // Switch to velocity input mode
            instance_->inputMode_ = 'v';
            instance_->menuPrinted_ = false;
            instance_->inputPos_ = 0;
            instance_->inputBuf_[0] = '\0';
            continue;
          } else if (mode == 'p' || mode == 'P') {
            // Switch to position input mode
            instance_->inputMode_ = 'p';
            instance_->menuPrinted_ = false;
            instance_->inputPos_ = 0;
            instance_->inputBuf_[0] = '\0';
            continue;
          }
        }
        // Invalid mode selection
        Serial.println();
        Serial.println("Invalid selection. Choose v, p, or z");
        Serial.print("Select mode - v for velocity, p for position, z to zero: ");
        instance_->inputPos_ = 0;
        instance_->inputBuf_[0] = '\0';
        continue;
        
      } else if (instance_->inputMode_ == 'v') {
        // Velocity value input
        char* endptr = nullptr;
        long val = strtol(instance_->inputBuf_, &endptr, 10);
        if (endptr != instance_->inputBuf_ && (*endptr == '\0')) {
          // clamp to -100..100 and publish to vref share
          if (val > 100) val = 100;
          if (val < -100) val = -100;
          if (instance_->vref_) {
            instance_->vref_->put(static_cast<int8_t>(val));
            Serial.print("[UITask] published velocity = "); Serial.println(val);
            // tell encoder/motor to start velocity run
            instance_->cmdShare_->put(static_cast<int8_t>(UITask::Command::VELOCITY_RUN));
          }
          
          // Clear any leftover characters in serial buffer to prevent immediate stops
          while (Serial.available() > 0) {
            Serial.read(); // Discard leftover characters
          }
          Serial.println("[UITask] Serial buffer cleared");
          
          instance_->menuPrinted_ = false;
          instance_->inputMode_ = 0; // Reset to mode selection
          return static_cast<int>(VELOCITY_RUN);
        } else {
          Serial.println();
          Serial.println("Invalid number, try again");
          Serial.print("Enter velocity (-100 to 100): ");
          instance_->inputPos_ = 0;
          instance_->inputBuf_[0] = '\0';
          continue;
        }
        
      } else if (instance_->inputMode_ == 'p') {
        // Position value input  
        char* endptr = nullptr;
        long val = strtol(instance_->inputBuf_, &endptr, 10);
        if (endptr != instance_->inputBuf_ && (*endptr == '\0')) {
          // Publish position reference to posref share
          if (instance_->posref_) {
            instance_->posref_->put(static_cast<int16_t>(val));
            Serial.print("[UITask] published position = "); Serial.println(val);
          } 
          // tell encoder/motor to start position run
          Serial.println("[UITask] Setting cmdShare to POSITION_RUN (2)"); 
          instance_->cmdShare_->put(static_cast<int8_t>(UITask::Command::POSITION_RUN));
          
          // Clear any leftover characters in serial buffer to prevent immediate stops
          while (Serial.available() > 0) {
            Serial.read(); // Discard leftover characters
          }
          Serial.println("[UITask] Serial buffer cleared");
          
          instance_->menuPrinted_ = false;
          instance_->inputMode_ = 0; // Reset to mode selection
          return static_cast<int>(POSITION_RUN);
        } else {
          Serial.println();
          Serial.println("Invalid number, try again");
          Serial.print("Enter position: ");
          instance_->inputPos_ = 0;
          instance_->inputBuf_[0] = '\0';
          continue;
        }
      }
    } else {
      // accumulate printable characters
      if (instance_->inputPos_ + 1 < sizeof(instance_->inputBuf_)) {
        if (ch >= ' ' && ch <= '~') instance_->inputBuf_[instance_->inputPos_++] = ch;
      }
    }
  }
  return static_cast<int>(WAIT_FOR_INPUT);
}

// VELOCITY_RUN: print velocity; allow switching/stopping
u_int8_t UITask::exec_velocityRun()
{
  if (!instance_) return -1;

  // Input handling (non-blocking)
  while (Serial.available() > 0) {
    const char ch = static_cast<char>(Serial.read());
    switch (ch) {
      case 'v': case 'V':
        // stop running and allow entering new velocity
        sendEncoderCmdStop();
        instance_->menuPrinted_ = false;
        return static_cast<int>(WAIT_FOR_INPUT);
      case 's': case 'S':
        // stop running and return to wait
        sendEncoderCmdStop();
        printHelpOnce();
        return static_cast<int>(WAIT_FOR_INPUT);
      case 'p': case 'P':
        // switch to position run
        sendEncoderCmdStop();
        instance_->menuPrinted_ = false;
        return static_cast<int>(WAIT_FOR_INPUT);
      case 'z': case 'Z':
        // zero encoder
        sendEncoderCmdZero();
        break;
      case 'h': case 'H': case '?':
        printHelpOnce();
        break;
      default:
        break;
      
    }
  }
  // Periodic value print
  const uint32_t now = millis();
  if (now - instance_->lastValuePrintMs_ >= 200) {
    instance_->lastValuePrintMs_ = now;
    float vel = 0.0f;
    float pos = 0.0f;
    if (instance_->velocityShare_) vel = instance_->velocityShare_->get();
    if (instance_->positionShare_) pos = instance_->positionShare_->get();
    Serial.print("velo: "); Serial.print(vel, 2);
    Serial.print(" , ");
    Serial.print("pos: "); Serial.println(pos, 2);
  }

  return static_cast<int>(VELOCITY_RUN);
}
  // Position_RUN: print position; allow switching/stopping
u_int8_t UITask::exec_positionRun()
{
  if (!instance_) return -1;

  // Input handling (non-blocking)
  
  while (Serial.available() > 0) {
    const char ch = static_cast<char>(Serial.read());
    switch (ch) {
      case 'v': case 'V':
        // stop running and allow entering new velocity
        sendEncoderCmdStop();
        instance_->menuPrinted_ = false;
        return static_cast<int>(WAIT_FOR_INPUT);
      case 's': case 'S':
        sendEncoderCmdStop();
        return static_cast<int>(WAIT_FOR_INPUT);
      case 'p': case 'P':
        // switch to position run
        sendEncoderCmdStop();
        instance_->menuPrinted_ = false;
        return static_cast<int>(WAIT_FOR_INPUT);
      case 'z': case 'Z':
        // zero encoder
        sendEncoderCmdZero();
        break;
      case 'h': case 'H': case '?':
        printHelpOnce();
        break;
      default:
        break;
    }
  }

  // Periodic value print
  const uint32_t now = millis();
  if (now - instance_->lastValuePrintMs_ >= 200) {
    instance_->lastValuePrintMs_ = now;
    float vel = 0.0f;
    float pos = 0.0f;
    if (instance_->velocityShare_) vel = instance_->velocityShare_->get();
    if (instance_->positionShare_) pos = instance_->positionShare_->get();
    Serial.print("velo: "); Serial.print(vel, 2);
    Serial.print(" , ");
    Serial.print("pos: "); Serial.println(pos, 2);
  }
  return static_cast<int>(POSITION_RUN);
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
    // Web server handles requests automatically
    // Broadcast telemetry periodically
    const unsigned long now = millis();
    if (now - lastTelemetryBroadcast_ >= telemetryInterval_) {
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
    Serial.println("No WiFi credentials provided - running in serial mode only");
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
    Serial.println("Failed to start Access Point. Running in serial mode only.");
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
  float vel = 0.0f, pos = 0.0f;
  if (velocityShare_) vel = velocityShare_->get();
  if (positionShare_) pos = positionShare_->get();
  
  // Temporary debug
  static uint32_t lastDebug = 0;
  if (millis() - lastDebug > 2000) {
    Serial.print("Telemetry: vel=");
    Serial.print(vel);
    Serial.print(", pos=");
    Serial.println(pos);
    lastDebug = millis();
  }
  
  // Get IMU data
  EulerAngles euler = {0, 0, 0};
  GyroData gyro = {0, 0, 0};
  AccelData accel = {0, 0, 0};
  
  if (eulerAngles_) euler = eulerAngles_->get();
  if (gyroData_) gyro = gyroData_->get();
  if (accelData_) accel = accelData_->get();
  
  String json = "{";
  json += "\"type\":\"telemetry\",";
  json += "\"velocity\":" + String(vel, 2) + ",";
  json += "\"position\":" + String(pos, 2) + ",";
  json += "\"euler\":[" + String(euler.x, 1) + "," + String(euler.y, 1) + "," + String(euler.z, 1) + "],";
  json += "\"gyro\":[" + String(gyro.x, 2) + "," + String(gyro.y, 2) + "," + String(gyro.z, 2) + "],";
  json += "\"accel\":[" + String(accel.x, 2) + "," + String(accel.y, 2) + "," + String(accel.z, 2) + "],";
  json += "\"timestamp\":" + String(millis());
  json += "}";
  
  return json;
}

void UITask::broadcastTelemetry()
{
  if (!ws_) {
    Serial.println("[UITask] ERROR: WebSocket is null!");
    return;
  }
  
  static uint32_t broadcastCount = 0;
  broadcastCount++;
  
  String message = createTelemetryMessage();
  ws_->textAll(message);
  
  if (broadcastCount % 10 == 0) {
    Serial.print("[UITask] Broadcast #");
    Serial.print(broadcastCount);
    Serial.print(", clients: ");
    Serial.println(ws_->count());
  }
}

void UITask::processWebSocketMessage(AsyncWebSocketClient* client, const String& message)
{
  // Parse JSON command from web interface
  // For now, implement basic commands
  if (message == "stop") {
    if (cmdShare_) cmdShare_->put(static_cast<int8_t>(Command::STOP));
    sendWebSocketResponse(client, "{\"status\":\"stopped\"}");
  }
  else if (message == "zero") {
    sendEncoderCmdZero();
    sendWebSocketResponse(client, "{\"status\":\"zeroed\"}");
  }
  else if (message.startsWith("velocity:")) {
    float vel = message.substring(9).toFloat();
    if (vref_) vref_->put(static_cast<int8_t>(vel));
    if (cmdShare_) cmdShare_->put(static_cast<int8_t>(Command::VELOCITY_RUN));
    sendWebSocketResponse(client, "{\"status\":\"velocity_set\",\"value\":" + String(vel) + "}");
  }
  else if (message.startsWith("position:")) {
    float pos = message.substring(9).toFloat();
    if (posref_) posref_->put(static_cast<int16_t>(pos));
    if (cmdShare_) cmdShare_->put(static_cast<int8_t>(Command::POSITION_RUN));
    sendWebSocketResponse(client, "{\"status\":\"position_set\",\"value\":" + String(pos) + "}");
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
