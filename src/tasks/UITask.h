/**
 * @file UITask.h
 * @brief Comprehensive web-based UI with WebSocket telemetry and mode management
 * 
 * This task provides the primary user interface for the motor control system via
 * a web browser. It implements a complete web server with real-time WebSocket
 * communication for telemetry data and command inputs. The task coordinates all
 * system modes and aggregates data from multiple sensors and subsystems.
 * 
 * Key Features:
 * - **Web Server**: Serves HTML/CSS/JS files from SPIFFS filesystem
 * - **WebSocket Server**: Real-time bidirectional communication for telemetry and commands
 * - **REST API**: HTTP endpoints for system status and control
 * - **Serial Interface**: Debug commands via USB serial (v, p, s, z, h/?)
 * - **Mode Management**: Coordinates system-wide operating mode transitions
 * - **Multi-Sensor Telemetry**: Aggregates motor, encoder, IMU, and camera data
 * 
 * Telemetry Data (sent via WebSocket):
 * - Motor positions and velocities (pan and tilt)
 * - IMU data (euler angles, gyroscope, accelerometer)
 * - Camera tracking errors (pan_error, tilt_error)
 * - System state and calibration status
 * 
 * Command Interface:
 * - Mode selection: MOTOR_TEST, TELEOP, TRACKER, CALIBRATE
 * - Motor velocity/position setpoints
 * - D-pad control inputs for manual operation
 * - Calibration triggers
 * 
 * FSM States:
 * - **CHOOSE_MODE**: Waiting for mode selection (UI_mode == 0)
 * - **CALIBRATE**: Running calibration sequence (dcalibrate == true)
 * - **MOTOR_TEST**: Motor testing with manual setpoints
 * - **TELEOP**: Manual control via D-pad inputs
 * - **TRACKER**: Automatic LED tracking mode
 * 
 * @author User Interface Team
 * @date November 2025
 * @version 3.2
 * 
 * @see UITask.cpp for WebSocket message handling and telemetry formatting
 * @see data/index.html for web interface implementation
 */
#pragma once
#include <Arduino.h>
#include "../fsm/FSM.h"
#include "../fsm/State.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "taskqueue.h"
#include "../fsm/State.h"
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

  enum StateId : uint8_t {
    CHOOSE_MODE = 0,  ///< Waiting for mode selection (UI_mode == 0)
    CALIBRATE = 4,    ///< Calibration mode (dcalibrate == true)
    MOTOR_TEST = 3,   ///< Motor testing with velocity/position control
    TELEOP = 2,       ///< Teleop mode (UI_mode == 2)
    TRACKER = 1       ///< Tracker mode (UI_mode == 1)
  };

  /**
   * @brief Construct UI task.
   * @param tilt_positionShare Tilt encoder position share
   * @param tilt_velocityShare Tilt encoder velocity share
   * @param tilt_vref Tilt motor velocity reference share
   * @param tilt_posref Tilt motor position reference share
   * @param tilt_cmdShare Tilt encoder command share
   * @param tilt_zeroShare Tilt encoder zero share
   * @param pan_positionShare Pan encoder position share
   * @param pan_velocityShare Pan encoder velocity share
   * @param pan_vref Pan motor velocity reference share
   * @param pan_posref Pan motor position reference share
   * @param pan_cmdShare Pan encoder command share
   * @param pan_zeroShare Pan encoder zero share
   * @param eulerAngles IMU euler angles share
   * @param gyroData IMU gyro data share
   * @param accelData IMU accel data share
   * @param updateMs UI update/scan period in ms
   */
  UITask(Share<float>* tilt_positionShare,
    Share<float>*   tilt_velocityShare,
    Share<int8_t>*  tilt_vref,
    Share<int16_t>*  tilt_posref,
    Share<uint8_t>*  tilt_cmdShare,
    Share<bool>*    tilt_zeroShare,
    Share<float>* pan_positionShare,
    Share<float>*   pan_velocityShare,
    Share<int8_t>*  pan_vref,
    Share<int16_t>*  pan_posref,
    Share<uint8_t>*  pan_cmdShare,
    Share<bool>*    pan_zeroShare,
    Share<int8_t>*  dpad_pan,
    Share<int8_t>*  dpad_tilt,
    Share<bool>*    imu_mode,
    Share<uint8_t>* ui_mode,
    Share<bool>*    motortest_mode,
    Share<bool>*    dcalibrate,
    Share<EulerAngles>* eulerAngles,
    Share<GyroData>* gyroData,
    Share<AccelData>* accelData,
    Share<bool>*    hasLed,
    Share<uint16_t>* ledThreshold,
    Share<int16_t>* pan_err,
    Share<int16_t>* tilt_err,
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
  
  // Telemetry queue accessors for encoder tasks to push data
  Queue<float>* getTiltVelQueue() { return tilt_vel_queue_; }
  Queue<float>* getTiltPosQueue() { return tilt_pos_queue_; }
  Queue<float>* getPanVelQueue() { return pan_vel_queue_; }
  Queue<float>* getPanPosQueue() { return pan_pos_queue_; }
  Queue<bool>* getHasLedQueue() { return has_led_queue_; }

  // Web interface accessors
  bool isWiFiConnected() const { return WiFi.status() == WL_CONNECTED; }
  String getIPAddress() const { return WiFi.localIP().toString(); }
  uint32_t getWebSocketClientCount() const;

private:
  // Tilt motor shares
  Share<float>* tilt_positionShare_ = nullptr;
  Share<float>* tilt_velocityShare_ = nullptr;
  Share<int8_t>* tilt_vref_ = nullptr;
  Share<int16_t>* tilt_posref_ = nullptr;
  Share<uint8_t>* tilt_cmdShare_ = nullptr;
  Share<bool>* tilt_zeroShare_ = nullptr;
  
  // Pan motor shares
  Share<float>* pan_positionShare_ = nullptr;
  Share<float>* pan_velocityShare_ = nullptr;
  Share<int8_t>* pan_vref_ = nullptr;
  Share<int16_t>* pan_posref_ = nullptr;
  Share<uint8_t>* pan_cmdShare_ = nullptr;
  Share<bool>* pan_zeroShare_ = nullptr;
  // Motor test mode share
  Share<bool>* motortest_mode_ = nullptr;
  
  // D-pad direction shares
  Share<int8_t>* dpad_pan_ = nullptr;
  Share<int8_t>* dpad_tilt_ = nullptr;
  
  // IMU mode control
  Share<bool>* imu_mode_ = nullptr;
  
  // UI mode and calibration control
  Share<uint8_t>* ui_mode_ = nullptr;
  Share<bool>* dcalibrate_ = nullptr;
  
  // IMU shares
  Share<EulerAngles>* eulerAngles_ = nullptr;
  Share<GyroData>* gyroData_ = nullptr;
  Share<AccelData>* accelData_ = nullptr;
  
  // Camera/LED tracking shares
  Share<bool>* hasLed_ = nullptr;
  Share<uint16_t>* ledThreshold_ = nullptr;
  Share<int16_t>* pan_err_ = nullptr;
  Share<int16_t>* tilt_err_ = nullptr;
  
  // Telemetry queues for non-blocking data transfer (encoder tasks push, UITask reads)
  Queue<float>* tilt_vel_queue_ = nullptr;
  Queue<float>* tilt_pos_queue_ = nullptr;
  Queue<float>* pan_vel_queue_ = nullptr;
  Queue<float>* pan_pos_queue_ = nullptr;
  Queue<bool>* has_led_queue_ = nullptr;
  
  // Legacy compatibility pointers (point to tilt for backward compatibility)
  Share<float>* positionShare_ = nullptr;
  Share<float>* velocityShare_ = nullptr;
  Share<int8_t>* vref_ = nullptr;
  Share<int16_t>* posref_ = nullptr;
  Share<uint8_t>* cmdShare_ = nullptr;

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
  
  // Cached telemetry values to avoid mutex blocking
  float cached_tilt_vel_ = 0.0f;
  float cached_tilt_pos_ = 0.0f;
  float cached_pan_vel_ = 0.0f;
  float cached_pan_pos_ = 0.0f;
  bool cached_has_led_ = false;
  int16_t cached_pan_err_ = 0;
  int16_t cached_tilt_err_ = 0;
  EulerAngles cached_euler_ = {0.0f, 0.0f, 0.0f};
  GyroData cached_gyro_ = {0.0f, 0.0f, 0.0f};
  AccelData cached_accel_ = {0.0f, 0.0f, 0.0f};
  unsigned long lastCacheUpdate_ = 0;

  // FSM
    // FSM + states
  State state_choose_mode_{CHOOSE_MODE, "CHOOSE_MODE", &UITask::exec_choose_mode};
  State state_calibrate_{CALIBRATE, "CALIBRATE", &UITask::exec_calibrate};
  State state_motor_test_{MOTOR_TEST, "MOTOR_TEST", &UITask::exec_motor_test};
  State state_teleop_{TELEOP, "TELEOP", &UITask::exec_teleop};
  State state_tracker_{TRACKER, "TRACKER", &UITask::exec_tracker};
  State* states_[5] = {&state_choose_mode_, &state_tracker_, &state_teleop_, &state_motor_test_, &state_calibrate_ };
  FSM fsm_;

  // State functions
  static uint8_t exec_choose_mode();
  static uint8_t exec_calibrate();
  static uint8_t exec_motor_test();
  static uint8_t exec_teleop();
  static uint8_t exec_tracker();
 

  // Web server methods
  bool initSPIFFS();
  void setupWebRoutes();
  void setupAPI();
  void connectToWiFi();
  void updateWebServer();
  void broadcastTelemetry();
  String createTelemetryMessage();
  void processWebSocketMessage(AsyncWebSocketClient* client, const String& message);
  static void sendWebSocketResponse(AsyncWebSocketClient* client, const String& response);
  
  // State-specific message handlers
  static void handleChooseModeMessage(AsyncWebSocketClient* client, const String& message);
  static void handleCalibrateMessage(AsyncWebSocketClient* client, const String& message);
  static void handleMotorTestMessage(AsyncWebSocketClient* client, const String& message);
  static void handleTeleopMessage(AsyncWebSocketClient* client, const String& message);
  static void handleTrackerMessage(AsyncWebSocketClient* client, const String& message);
  
  // Pending message queue
  static String pendingMessage_;
  static AsyncWebSocketClient* pendingClient_;
  
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

