/**
 * @file UITask.h
 * @brief Comprehensive user interface task with web server, WebSocket, and serial control
 * 
 * @author User Interface Team
 * @date November 2025
 * @version 3.1
 * 
 */
#pragma once
#include <Arduino.h>
#include "FSM.h"
#include "State.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "taskqueue.h"
#include "State.h"
#include "taskshare.h"  // Share<T>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <SPIFFS.h>
#include "IMU_TASK.h"

// Forward declarations for IMU data structures
struct EulerAngles;
struct GyroData;
struct AccelData;

/**
 * @brief UI task with web server, WebSocket, and serial interface
 *
 * Features:
 * - Serial commands: v, p, s, z, h/?
 * - Web server: serves static files from SPIFFS
 * - WebSocket: real-time bidirectional communication
 * - REST API: HTTP endpoints for status/commands
 */
class UITask {
public:
  /// Commands that can be casted to queue
    enum class Command {     
        STOP = 0,       ///< Stop encoder readings
        VELOCITY_RUN = 1,         ///< Start encoder readings
        POSITION_RUN = 2,///< Start position run
        ZERO = 3       ///< Reset position to zero
    };

  enum StateId : uint8_t {
    WAIT_FOR_INPUT = 0,
    VELOCITY_RUN   = 1,
    POSITION_RUN   = 2,
  };

  /**
   * @brief Construct UI task.
   * @param positionShare Pointer to position share (may be nullptr)
   * @param velocityShare Pointer to velocity share (may be nullptr)
   * @param encoderCmdQueue Queue handle to send commands to EncoderTask (may be nullptr)
   * @param updateMs UI update/scan period in ms
   */
  UITask(Share<float>* positionShare,
    Share<float>*   velocityShare,
    Share<int8_t>*  vref,
    Share<int16_t>*  posref,
    Share<int8_t>*  cmdShare,
    Share<EulerAngles>* eulerAngles,
    Share<GyroData>* gyroData,
    Share<AccelData>* accelData,
    uint32_t        updateMs = 200) noexcept;

  /**
   * @brief Initialize web server and WiFi
   * @param ssid WiFi network name
   * @param password WiFi password
   * @return true if successful
   */
  bool initWebServer(const String& ssid = "", const String& password = "");

  void start(uint8_t priority = 1, int8_t core = 1);
  void update() noexcept;
  // FreeRTOS entry function is defined in the .cpp as a C-linkage function
  // extern "C" void ui_task_func(void* pvParameters);

  // Public accessors used by the C-style task entry
  uint32_t get_updateMs() const noexcept { return updateMs_; }
  static void set_instance(UITask* inst) { instance_ = inst; }

  // Web interface accessors
  bool isWiFiConnected() const { return WiFi.status() == WL_CONNECTED; }
  String getIPAddress() const { return WiFi.localIP().toString(); }
  uint32_t getWebSocketClientCount() const;

private:
  // Shares / queue
  Share<float>* positionShare_ = nullptr;
  Share<float>*   velocityShare_ = nullptr;
  Share<int8_t>*  vref_ = nullptr;
  Share<int8_t>*  cmdShare_ = nullptr;
  Share<int16_t>*  posref_ = nullptr;
  Share<EulerAngles>* eulerAngles_ = nullptr;
  Share<GyroData>* gyroData_ = nullptr;
  Share<AccelData>* accelData_ = nullptr;

  // Timing
  uint32_t updateMs_ = 200;
  uint32_t lastMenuPrintMs_ = 0;
  uint32_t lastValuePrintMs_ = 0;
  // Whether we've printed the menu since entering WAIT state
  bool menuPrinted_ = false;
  // Input buffer for numeric entry from Serial
  char inputBuf_[32];
  size_t inputPos_ = 0;
  // Input mode: 0=selecting mode, 'v'=velocity input, 'p'=position input
  char inputMode_ = 0;

  // Web server components
  AsyncWebServer* server_ = nullptr;
  AsyncWebSocket* ws_ = nullptr;
  String ssid_;
  String password_;
  unsigned long lastConnectAttempt_ = 0;
  unsigned long lastTelemetryBroadcast_ = 0;
  bool connecting_ = false;
  const unsigned long telemetryInterval_ = 200; // 200ms = 5Hz

  // FSM
    // FSM + states
  State state_wait_{0, "WAIT_FOR_INPUT", &UITask::exec_waitForInput};
  State state_velocity_run_{1, "VELOCITY_RUN", &UITask::exec_velocityRun};
  State state_position_run_{2, "POSITION_RUN", &UITask::exec_positionRun};
  State* states_[3] = { &state_wait_, &state_velocity_run_, &state_position_run_ };
  FSM fsm_;

  // State functions
  static uint8_t exec_waitForInput();
  static uint8_t exec_velocityRun();
  static uint8_t exec_positionRun();

  // Helpers
  static void printHelpOnce();
  static void sendEncoderCmdZero();
  static void sendEncoderCmdStart();
  static void sendEncoderCmdStop();

  // Web server methods
  bool initSPIFFS();
  void setupWebRoutes();
  void setupAPI();
  void connectToWiFi();
  void updateWebServer();
  void broadcastTelemetry();
  String createTelemetryMessage();
  void processWebSocketMessage(AsyncWebSocketClient* client, const String& message);
  void sendWebSocketResponse(AsyncWebSocketClient* client, const String& response);
  
  // Static WebSocket event handler
  static void onWebSocketEvent(AsyncWebSocket *server, 
                              AsyncWebSocketClient *client, 
                              AwsEventType type, 
                              void *arg, 
                              uint8_t *data, 
                              size_t len);

  // Singleton for static callbacks
  static UITask* instance_;
};

