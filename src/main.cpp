/**
 * @file main.cpp
 * @brief Main application entry point for ESP32 camera-guided pan-tilt tracking system
 * 
 * This file implements a sophisticated multi-axis tracking system combining computer vision,
 * inertial measurement, and PID motor control. The system can automatically track LED targets
 * using camera feedback, provide manual teleoperation via web interface, and stream real-time
 * telemetry data including motor positions, IMU orientation, and tracking errors.
 * 
 * System Architecture:
 * -------------------
 * The system uses FreeRTOS multitasking to coordinate seven concurrent tasks:
 * 
 * 1. **MotorTask (Tilt & Pan)**: Multi-mode motor control (velocity, position, camera-based)
 * 2. **EncoderTask (Tilt & Pan)**: High-frequency position/velocity measurement via PCNT
 * 3. **ControllerTask**: Top-level FSM coordinating tracking modes and state transitions
 * 4. **CameraTask**: LED detection and visual servoing error computation
 * 5. **IMUTask**: BNO055 9-DOF orientation and motion sensing
 * 6. **UITask**: Web server, WebSocket telemetry, and command interface
 * 
 * Hardware Components:
 * -------------------
 * - ESP32-WROVER microcontroller (dual-core, WiFi, 8MB PSRAM)
 * - 2× DRV8833 dual H-bridge motor drivers
 * - 2× DC motors with 70:1 gearboxes
 * - 2× Quadrature encoders (64 CPR)
 * - OV5640 camera module (5MP, autofocus)
 * - BNO055 9-DOF IMU (accelerometer, gyroscope, magnetometer)
 * - Pan-tilt mechanical platform
 * 
 * Control Modes:
 * -------------
 * - **TRACKER**: Automatic LED tracking using camera feedback
 * - **TELEOP**: Manual control via web UI D-pad
 * - **MOTOR_TEST**: Open-loop motor testing for diagnostics
 * - **CALIBRATE**: Encoder homing and calibration sequence
 * 
 * PID Tuning Guide:
 * -----------------
 * Four independent PID controllers are available for tuning:
 * 
 * 1. **tilt_velocity_pid** - Tilt axis velocity control
 *    - Current: Kp=0.10, Ki=0.0, Kd=0.0
 *    - Tuning: Increase Kp for faster velocity response, add Ki to eliminate steady-state error
 * 
 * 2. **tilt_position_pid** - Tilt axis position control  
 *    - Current: Kp=0.080, Ki=0.0005, Kd=0.0
 *    - Tuning: Increase Kp for stiffer position hold, add Kd to reduce overshoot
 * 
 * 3. **pan_velocity_pid** - Pan axis velocity control
 *    - Current: Kp=0.10, Ki=0.0, Kd=0.0
 *    - Tuning: Adjust independently from tilt due to different mechanical characteristics
 * 
 * 4. **pan_position_pid** - Pan axis position control
 *    - Current: Kp=0.080, Ki=0.0005, Kd=0.0
 *    - Tuning: May need different gains than tilt due to gravity effects
 * 
 * To adjust gains, modify the Init() calls in setup():
 * ```cpp
 * tilt_velocity_pid.Init(Kp, Ki, Kd);
 * ```
 * 
 * Network Configuration:
 * ---------------------
 * - WiFi SSID/Password configured in UITask.cpp
 * - Web interface accessible at: http://<ESP32_IP_ADDRESS>
 * - WebSocket telemetry on same port as HTTP server
 * - Serial debug output at 115200 baud
 * 
 * Pin Assignments:
 * ---------------
 * See hardware configuration section in setup() for complete GPIO mapping:
 * - Motors: GPIO pins for DRV8833 IN1/IN2
 * - Encoders: GPIO pins for Phase A/B quadrature signals
 * - Camera: SPIO interface to OV5640
 * - IMU: I2C interface to BNO055
 * 
 * @author Motor Control Development Team
 * @date December 2025
 * @version 2.7
 * 
 * @see MotorTask.h for motor control implementation
 * @see CameraTask.h for computer vision details
 * @see ControllerTask.h for FSM state descriptions
 */

// ===== ESP32 Pan and Tilt Motor Control System with IMU Integration =====
#include <Arduino.h>
#include <cstdint>
#include "hardware/DRV883.h"
#include "tasks/MotorTask.h"
#include "hardware/Encoder.h"
#include "tasks/EncoderTask.h"
#include "tasks/UITask.h"
#include "tasks/IMU_Task.h"
#include <Wire.h>
#include "hardware/Adafruit_BNO055.h"
#include "tasks/CameraTask.h"
#include "tasks/ControllerTask.h"
#include "hardware/OV5640_camera.h"
// FreeRTOS APIs used to create tasks/queues
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
// ME507 share utility
#include "taskshare.h"

// FreeRTOS task entry declared in MotorTask.cpp
extern "C" void ui_task_func(void* arg);
extern "C" void imu_task_func(void* arg);
extern "C" void controller_task_func(void* arg);
extern "C" void camera_task_func(void* arg);

/**
 * @brief FreeRTOS task entry point for tilt motor control
 * 
 * Dedicated task function for tilt axis motor control to avoid static instance
 * conflicts between pan and tilt MotorTask objects. Sets the static instance
 * pointer before each update to ensure correct object context.
 * 
 * Task Configuration:
 * - Update rate: 50ms (20Hz)
 * - Priority: Set during xTaskCreate() call
 * - Stack size: Configured in task creation
 * 
 * @param arg Pointer to tilt MotorTask object (cast from void*)
 * 
 * @note Separate task functions required because MotorTask uses static instance pattern
 * @note Instance pointer set before each update() call for correct FSM execution
 */
extern "C" void tilt_motor_task_func(void* arg) {
  MotorTask* motorTask = static_cast<MotorTask*>(arg);
  const TickType_t delayTicks = pdMS_TO_TICKS(50);
  for (;;) {
    if (motorTask) {
      MotorTask::set_instance(motorTask);  // Set instance before each update
      motorTask->update();
    }
    vTaskDelay(delayTicks);
  }
}

/**
 * @brief FreeRTOS task entry point for pan motor control
 * 
 * Dedicated task function for pan axis motor control, independent from tilt axis.
 * Maintains separate static instance context to prevent conflicts during concurrent
 * execution of both motor control tasks.
 * 
 * Task Configuration:
 * - Update rate: 50ms (20Hz)
 * - Priority: Set during xTaskCreate() call
 * - Stack size: Configured in task creation
 * 
 * @param arg Pointer to pan MotorTask object (cast from void*)
 * 
 * @note Must be separate from tilt_motor_task_func due to static instance pattern
 * @note Both motor tasks run concurrently without interference
 */
extern "C" void pan_motor_task_func(void* arg) {
  MotorTask* motorTask = static_cast<MotorTask*>(arg);
  const TickType_t delayTicks = pdMS_TO_TICKS(50);
  for (;;) {
    if (motorTask) {
      MotorTask::set_instance(motorTask);  // Set instance before each update
      motorTask->update();
    }
    vTaskDelay(delayTicks);
  }
}

/**
 * @brief FreeRTOS task entry point for tilt encoder monitoring
 * 
 * High-frequency task for tilt axis encoder position and velocity measurement.
 * Runs at 100Hz to minimize control loop latency and ensure accurate velocity
 * calculation via the Encoder's sliding window algorithm.
 * 
 * Task Configuration:
 * - Update rate: 10ms (100Hz)
 * - Priority: Typically higher than motor tasks for accurate feedback
 * - Stack size: Configured in task creation
 * 
 * @param arg Pointer to tilt EncoderTask object (cast from void*)
 * 
 * @note Fast update rate critical for control loop performance
 * @note Separate from pan encoder to avoid instance conflicts
 */
extern "C" void tilt_encoder_task_func(void* arg) {
  EncoderTask* encoderTask = static_cast<EncoderTask*>(arg);
  const TickType_t delayTicks = pdMS_TO_TICKS(10);
  for (;;) {
    if (encoderTask) {
      EncoderTask::set_instance(encoderTask);  // Set instance before each update
      encoderTask->update();
    }
    vTaskDelay(delayTicks);
  }
}

/**
 * @brief FreeRTOS task entry point for pan encoder monitoring
 * 
 * High-frequency task for pan axis encoder position and velocity measurement.
 * Operates independently from tilt encoder to provide concurrent feedback for
 * both axes without static instance conflicts.
 * 
 * Task Configuration:
 * - Update rate: 10ms (100Hz)
 * - Priority: Typically higher than motor tasks for accurate feedback
 * - Stack size: Configured in task creation
 * 
 * @param arg Pointer to pan EncoderTask object (cast from void*)
 * 
 * @note Matches tilt encoder update rate for consistent control behavior
 * @note Separate task function prevents static instance pointer conflicts
 */
extern "C" void pan_encoder_task_func(void* arg) {
  EncoderTask* encoderTask = static_cast<EncoderTask*>(arg);
  const TickType_t delayTicks = pdMS_TO_TICKS(10);
  for (;;) {
    if (encoderTask) {
      EncoderTask::set_instance(encoderTask);  // Set instance before each update
      encoderTask->update();
    }
    vTaskDelay(delayTicks);
  }
}

// ----------------Pin Declarations ----------------
// ----------------Pan Motor pins ----------------
#define PanIN1 GPIO_NUM_2
#define PanIN2 GPIO_NUM_5
#define nSLP_PIN GPIO_NUM_13
// ---------------- Pan Encoder pins/unit ----------------
#define TiltENCODER_A_GPIO GPIO_NUM_14
#define TiltENCODER_B_GPIO GPIO_NUM_19
#define TiltENCODER_UNIT   PCNT_UNIT_0
// ----------------Tilt Motor pins ----------------
#define TiltIN1 GPIO_NUM_18
#define TiltIN2 GPIO_NUM_23
// ---------------- Tilt Encoder pins/unit ----------------
#define PanENCODER_A_GPIO GPIO_NUM_3
#define PanENCODER_B_GPIO GPIO_NUM_1 //should be GPI01
#define PanENCODER_UNIT   PCNT_UNIT_1
// --------------------------------------------------

// ---------------- Hardware Objects ----------------
//Camera Object 
OV5640Camera camera = OV5640Camera(760);
// Motor: IN1/IN2 on LEDC channels 0 and 1
DRV883 panmotor(PanIN1, PanIN2, /*LEDC chs*/ 0, 1);
Encoder panencoder(PanENCODER_A_GPIO, PanENCODER_B_GPIO, PanENCODER_UNIT);
DRV883 tiltmotor(TiltIN1, TiltIN2, /*LEDC chs*/ 2, 3);
Encoder tiltencoder(TiltENCODER_A_GPIO, TiltENCODER_B_GPIO, TiltENCODER_UNIT);
// IMU instances (global so they can be accessed from loop)
TwoWire I2C = TwoWire(0);
Adafruit_BNO055 bno055_sensor = Adafruit_BNO055(1234, 0x28, &I2C);
// // --------------------------------------------------

// ---------------- Shares and Queues ----------------
//Camera Shares:
Share<int16_t> pan_err{"pan_err"};
Share<int16_t> tilt_err{"tilt_err"};
Share<bool> hasLed{"hasLed"};
Share<uint16_t> ledThreshold{"ledThreshold"};
Share<uint8_t> cam_Mode{"camMode"};
Share<uint8_t> UI_mode{"UI_mode"};
Share<bool> dcalibrate{"dcalibrate"};
// Encoder telemetry shares (declare BEFORE tasks that reference them)
Share<float> tilt_encPosition{"tiltenc_pos"};
Share<float> tilt_encVelocity{"tiltenc_vel"};
Share<float> pan_encPosition{"panenc_pos"};
Share<float> pan_encVelocity{"panenc_vel"};
// Motor reference (0-100)
// Motor reference (-100..100)
Share<int8_t> tilt_vref{"tiltvref"};
Share<int16_t> tilt_posref{"tiltposref"};
Share<int8_t> pan_vref{"panvref"};
Share<int16_t> pan_posref{"panposref"};
// Shared command for encoder/motor (0=STOP, 1=VELOCITY_RUN, 2=POSITION_RUN)
Share<uint8_t> tilt_Cmd{"tiltenc_cmd"};
Share<uint8_t> pan_Cmd{"panenc_cmd"};
// Shared zero flags for encoders
Share<bool> tilt_encZero{"tiltenc_zero"};
Share<bool> pan_encZero{"panenc_zero"};
// D-pad direction shares (-1, 0, 1)
Share<int8_t> dpad_pan{"dpad_pan"};
Share<int8_t> dpad_tilt{"dpad_tilt"};
// Motor test mode control (true = velocity control, false = position control)
Share<bool> motortest_mode{"motortest_mode"};
// IMU mode control (true = send IMU data, false = don't send)
Share<bool> imu_mode{"imu_mode"};
//shares for IMU
// where the 
Share<EulerAngles> g_eulerAngles{"euler_angles"};
Share<GyroData> g_gyroData{"gyro_data"};
Share<AccelData> g_accelData{"accel_data"};
// --------------------------------------------------

// ----------------   PID Objects   ----------------
// Create PID objects for independent gain tuning
PID tilt_velocity_pid;
PID tilt_position_pid;
PID pan_velocity_pid;
PID pan_position_pid;
// --------------------------------------------------

// ----------------   Task Objects   ----------------
//Camera task
CameraTask cameraTask(&camera, &pan_err, &tilt_err, &hasLed, &ledThreshold, &cam_Mode);
//Controller task
ControllerTask controllerTask(&pan_err, &tilt_err, &tilt_encPosition, &tilt_vref, &pan_vref, &tilt_Cmd, &pan_Cmd, &cam_Mode, &hasLed, &UI_mode, &dcalibrate, &dpad_pan, &dpad_tilt);
// MotorTask/EncoderTask/UITask instances (pass pointers to the above shares/queue)
MotorTask tiltmotorTask(&tiltmotor, &tilt_Cmd, &tilt_encVelocity, &tilt_vref, &tilt_encPosition, &tilt_posref, &tilt_velocity_pid, &tilt_position_pid, &tilt_err, true); 
EncoderTask tiltencoderTask(&tiltencoder, &tilt_encPosition, &tilt_encVelocity, &tilt_Cmd, &tilt_encZero);
MotorTask panmotorTask(&panmotor, &pan_Cmd, &pan_encVelocity, &pan_vref, &pan_encPosition, &pan_posref, &pan_velocity_pid, &pan_position_pid, &pan_err, false);
EncoderTask panencoderTask(&panencoder, &pan_encPosition, &pan_encVelocity, &pan_Cmd, &pan_encZero);
// MotorTask/EncoderTask/UITask instances (pass pointers to the above shares/queue)
UITask uiTask(&tilt_encPosition, &tilt_encVelocity, &tilt_vref, &tilt_posref, &tilt_Cmd, &tilt_encZero,
  &pan_encPosition, &pan_encVelocity, &pan_vref, &pan_posref, &pan_Cmd, &pan_encZero,
  &dpad_pan, &dpad_tilt, &imu_mode, &UI_mode, &motortest_mode, &dcalibrate, &g_eulerAngles, &g_gyroData, &g_accelData,
  &hasLed, &ledThreshold, &pan_err, &tilt_err);
IMUTask imuTask(&bno055_sensor, &g_eulerAngles, &g_gyroData, &g_accelData);

// ---------------------------------------------------

/**
 * @brief Arduino setup() - System initialization and FreeRTOS task creation
 * 
 * This function performs complete system initialization including hardware configuration,
 * communication interface setup, PID controller tuning, and FreeRTOS task creation.
 * All initialization must complete before tasks begin execution.
 * 
 * Initialization Sequence:
 * 1. Initialize all Share<T> objects to safe default values
 * 2. Configure serial communication (115200 baud)
 * 3. Connect telemetry queues between tasks
 * 4. Initialize I2C bus for IMU communication
 * 5. Initialize BNO055 IMU sensor and load calibration
 * 6. Initialize OV5640 camera module
 * 7. Enable motor drivers (nSLP pin HIGH)
 * 8. Configure PID controller gains for all four controllers
 * 9. Initialize WiFi Access Point and web server
 * 10. Create seven FreeRTOS tasks with appropriate priorities
 * 
 * Task Priority Scheme:
 * - Priority 5: Encoder tasks (highest - critical for control feedback)
 * - Priority 4: Camera task (high - visual servoing feedback)
 * - Priority 3: Controller task (medium-high - coordination)
 * - Priority 2: Motor tasks (medium - actuation)
 * - Priority 1: UI task (low - telemetry and commands)
 * - Priority 0: IMU task (lowest - supplementary data)
 * 
 * Hardware Configuration:
 * - I2C Bus: GPIO21 (SDA), GPIO22 (SCL) for IMU
 * - WiFi: Access Point mode "ESP32-MotorControl" / "motor123"
 * - Motor Enable: GPIO13 (nSLP_PIN) set HIGH
 * - Serial: 115200 baud for debug output
 * 
 * @note GPIO1 used by pan encoder (no USB Serial transmission)
 * @note Camera/IMU failures logged but don't halt system startup
 * @note All tasks start immediately after creation
 * @note setup() runs once on ESP32 core 1 before scheduler starts
 */
void setup() {
  // -------------------------------------------------------------------------
  // Step 1: Initialize All Share<T> Objects to Safe Defaults
  // -------------------------------------------------------------------------
  
  // Initialize all shares and queues
  pan_err.put(0);
  tilt_err.put(0);
  hasLed.put(false);
  ledThreshold.put(760);
  cam_Mode.put(0);
  UI_mode.put(0);
  dcalibrate.put(false);
  tilt_encPosition.put(0.0f);
  tilt_encVelocity.put(0.0f);
  pan_encPosition.put(0.0f);
  pan_encVelocity.put(0.0f);
  tilt_vref.put(0);
  tilt_posref.put(0);
  pan_vref.put(0);
  pan_posref.put(0);
  tilt_Cmd.put(0);
  pan_Cmd.put(0);
  tilt_encZero.put(false);
  pan_encZero.put(false);
  dpad_pan.put(0);
  dpad_tilt.put(0);
  motortest_mode.put(true);
  imu_mode.put(false);
  EulerAngles zero_angles = {0.0f, 0.0f, 0.0f};
  g_eulerAngles.put(zero_angles);
  GyroData zero_gyro = {0.0f, 0.0f, 0.0f};
  g_gyroData.put(zero_gyro);
  AccelData zero_accel = {0.0f, 0.0f, 0.0f};
  g_accelData.put(zero_accel);
  
  // -------------------------------------------------------------------------
  // Step 2: Initialize Serial Communication
  // -------------------------------------------------------------------------
  
  // Don't initialize Serial (USB) because GPIO1 is used for pan encoder
  Serial.begin(115200);
  delay(300);
  
  // -------------------------------------------------------------------------
  // Step 3: Connect Telemetry Queues Between Tasks
  // -------------------------------------------------------------------------
  
  // Connect encoder tasks to UITask telemetry queues
  tiltencoderTask.setTelemetryQueues(uiTask.getTiltVelQueue(), uiTask.getTiltPosQueue());
  panencoderTask.setTelemetryQueues(uiTask.getPanVelQueue(), uiTask.getPanPosQueue());
  // Connect camera task to UITask telemetry queue
  cameraTask.setTelemetryQueue(uiTask.getHasLedQueue());
  
  // -------------------------------------------------------------------------
  // Step 4: Serial Debug Messages
  // -------------------------------------------------------------------------
  
  // // Use Serial for all debug output instead (RX=GPIO0, TX=GPIO12)
  // Serial.begin(115200, SERIAL_8N1, 0, 12);
  delay(100);
  Serial.println("=== System Starting ===");
  Serial.println("Note: Using Serial for debug (GPIO1 used by pan encoder)");
  
  // -------------------------------------------------------------------------
  // Step 5: Initialize I2C Bus and IMU Sensor
  // -------------------------------------------------------------------------
  
  // Set I2C pins for ESP32
  I2C.setPins(21, 22);
  Serial.println("=== SporTrackr IMU Data Reader ===");
  Serial.println("Initializing IMU...");
  
  // Initialize and start IMU task
  if (imuTask.initIMU()) {
    Serial.println("IMU initialized successfully!");
    delay(300);
    
    // Load saved calibration offsets automatically
    Serial.println("Loading saved calibration offsets...");
    imuTask.loadSavedCalibration();
  } else {
    Serial.println("WARNING: IMU initialization failed!");
    Serial.println("Continuing without IMU...");
  }
  
  // -------------------------------------------------------------------------
  // Step 6: Initialize OV5640 Camera Module
  // -------------------------------------------------------------------------
  
  // Camera initialization - skip if it fails (GPIO conflicts)
  bool camera_ok = camera.begin();
  if (!camera_ok) {
    Serial.println("WARNING: Camera initialization failed!");
    Serial.println("Continuing without camera...");
  } else {
    Serial.println("Camera initialized successfully");
  }
  
  // -------------------------------------------------------------------------
  // Step 7: Enable Motor Drivers
  // -------------------------------------------------------------------------
  
  // Enable motors by setting nSLP (not-sleep) pin HIGH
  pinMode(nSLP_PIN, OUTPUT);
  digitalWrite(nSLP_PIN, HIGH);
  
  // -------------------------------------------------------------------------
  // Step 8: Initialize PID Controllers with Tuned Gains
  // -------------------------------------------------------------------------
  
  // Initialize PID controllers with tuned gains
  // Tilt motor PIDs
  tilt_velocity_pid.Init(0.30, 0.0, 0.0);      // Kp=0.10 for velocity control
  tilt_position_pid.Init(0.150, 0.005, 0.0);  // Kp=0.080, Ki=0.0005 for position control
  
  // Pan motor PIDs
  pan_velocity_pid.Init(0.10, 0.0, 0.0);       // Kp=0.10 for velocity control
  pan_position_pid.Init(0.150, 0.005, 0.0);   // Kp=0.080, Ki=0.0005 for position control
  
  Serial.println("PID controllers initialized");
  
  // -------------------------------------------------------------------------
  // Step 9: Initialize WiFi Access Point and Web Server
  // -------------------------------------------------------------------------
  
  // Initialize ESP32 as WiFi Access Point (much easier than connecting to existing networks!)
  // The ESP32 will create its own hotspot that you can connect to from any device
  uiTask.initWebServer("ESP32-MotorControl", "motor123");
  Serial.println("UITask web server initialized as Access Point");

  // -------------------------------------------------------------------------
  // Step 10: Create FreeRTOS Tasks
  // -------------------------------------------------------------------------
  
  // Start Motor task (50 ms tick in its own entry function)
  // Start Tilt Motor task
  xTaskCreate(
    tilt_motor_task_func,
    "TiltMotorTask",
    2048,
    &tiltmotorTask,
    2,
    nullptr
  );
  // Start Tilt Encoder task
  xTaskCreate(
    tilt_encoder_task_func,
    "TiltEncoderTask",
    2048,
    &tiltencoderTask,
    5,
    nullptr
  );
  // Start Pan Motor task
  xTaskCreate(
    pan_motor_task_func,
    "PanMotorTask",
    2048,
    &panmotorTask,
    2,
    nullptr
  );
  // Start Pan Encoder task
  xTaskCreate(
    pan_encoder_task_func,
    "PanEncoderTask",
    2048,
    &panencoderTask,
    5,
    nullptr
  );
  // Start UI task
  xTaskCreate(
    ui_task_func,
    "UITask",
    8192,  // Increased from 4096 for WebSocket telemetry broadcasting
    &uiTask,
    1,
    nullptr
  );
  //Start IMU task (disabled - I2C errors)
  xTaskCreate(
    imu_task_func,
    "IMUTask",
    4096,
    &imuTask,
    0,
    nullptr
  );
  // Start Controller task
  xTaskCreate(
    controller_task_func,
    "ControllerTask",
    2048,
    &controllerTask,
    3,
    nullptr
  );
  //Start Camera task (temporarily disabled to debug GPIO conflict)
  xTaskCreate(
    camera_task_func,
    "CameraTask",
    4096,
    &cameraTask,
    4,
    nullptr
  );

 }

/**
 * @brief Arduino loop() - Idle function for FreeRTOS operation
 * 
 * In a FreeRTOS-based system, the Arduino loop() function is not used for
 * main program logic. All application code runs in FreeRTOS tasks created
 * in setup(). The loop() function simply delays indefinitely to prevent
 * watchdog timer resets.
 * 
 * Task Execution:
 * - All seven tasks run concurrently under FreeRTOS scheduler
 * - Task priorities determine execution order when multiple tasks ready
 * - vTaskDelay() in each task yields CPU to other tasks
 * - FreeRTOS idle task runs when no other tasks ready (manages power)
 * 
 * @note This function runs on ESP32 core 1 (setup/loop core)
 * @note FreeRTOS scheduler manages all task execution timing
 * @note Infinite delay prevents watchdog timer from triggering
 * @note To modify system behavior, edit task code, not loop()
 */
void loop() {
  // Idle; tasks run under FreeRTOS
  delay(600000000);
}
