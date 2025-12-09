/**
 * @file IMU_TASK.h
 * @brief FreeRTOS task for BNO055 9-DOF IMU sensor data acquisition and distribution
 * 
 * This header defines a high-performance IMU task that continuously reads orientation,
 * angular velocity, and acceleration data from the Adafruit BNO055 sensor. The task
 * operates in NDOF (Nine Degrees of Freedom) fusion mode, providing pre-filtered and
 * calibrated sensor data to the rest of the system via thread-safe Share<T> objects.
 * 
 * Key Features:
 * - Continuous data streaming mode for real-time telemetry
 * - NDOF sensor fusion (accelerometer + gyroscope + magnetometer)
 * - Configurable sampling rate with millisecond precision
 * - Built-in calibration status monitoring and persistence
 * - FSM-based architecture for clean state management
 * - Thread-safe data distribution to multiple consumers
 * 
 * Data Outputs:
 * - Euler Angles: Roll, pitch, yaw in degrees (absolute orientation)
 * - Gyroscope: Angular velocity in degrees/second
 * - Accelerometer: Linear acceleration in m/s²
 * 
 * @author IMU Integration Team
 * @date November 2025
 * @version 1.8
 * 
 * @see IMU_TASK.cpp for implementation details and timing configuration
 * @see Adafruit_BNO055.h for sensor hardware interface
 */

#pragma once
#include <Arduino.h>
#include <cstdint>
#include "../fsm/FSM.h"
#include "../fsm/State.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "../hardware/Adafruit_BNO055.h"
#include "taskshare.h"

/**
 * @struct EulerAngles
 * @brief Container for absolute orientation in 3D space
 * @note All angles in degrees, right-hand coordinate system
 */
struct EulerAngles {
  float_t x;  ///< Roll angle (rotation about X-axis)
  float_t y;  ///< Pitch angle (rotation about Y-axis)
  float_t z;  ///< Yaw angle (rotation about Z-axis)
};

/**
 * @struct GyroData
 * @brief Container for angular velocity measurements
 * @note All rates in degrees/second, right-hand coordinate system
 */
struct GyroData {
  float_t x;  ///< Angular velocity about X-axis (roll rate)
  float_t y;  ///< Angular velocity about Y-axis (pitch rate)
  float_t z;  ///< Angular velocity about Z-axis (yaw rate)
};

/**
 * @struct AccelData
 * @brief Container for linear acceleration measurements
 * @note All accelerations in m/s², includes gravity component
 */
struct AccelData {
  float_t x;  ///< Acceleration along X-axis
  float_t y;  ///< Acceleration along Y-axis
  float_t z;  ///< Acceleration along Z-axis
};

/**
 * @class IMUTask
 * @brief Continuous IMU sensor data acquisition task with FSM control
 *
 * Manages the BNO055 IMU sensor in a dedicated FreeRTOS task, providing
 * continuous streaming of orientation, angular velocity, and acceleration data.
 * The task operates in SEND_DATA state continuously for real-time telemetry updates.
 * 
 * FSM States:
 * - **WAIT**: Initial state, transitions immediately to SEND_DATA after first sample
 * - **SEND_DATA**: Continuous data acquisition and publication (does not return to WAIT)
 * 
 * Thread Safety:
 * - All data outputs use Share<T> for atomic read/write operations
 * - Safe to access from multiple consumer tasks
 * - No mutex required for reading published data
 * 
 * Calibration:
 * - Supports runtime calibration status monitoring
 * - Can save/restore calibration offsets for faster startup
 * - Checks for minimum magnetic calibration before trusting data
 * 
 * @note Uses static instance pattern for FSM callback integration
 * @note Sample rate configurable but limited by I2C bus speed (~100Hz max practical)
 * @see performCalibration() for calibration procedure
 */
class IMUTask {
public:
  /**
   * @enum StateId
   * @brief FSM state identifiers for IMU task operation
   */
  enum StateId : uint8_t {
    WAIT = 0,       ///< Initial state, waits for first sample interval
    SEND_DATA = 1,  ///< Continuous data acquisition and publishing
  };

  /**
   * @brief Construct a new IMU Task object
   * 
   * @param imu Pointer to initialized Adafruit_BNO055 sensor object
   * @param EulerAngles Pointer to shared euler angle data container
   * @param GyroData Pointer to shared gyroscope data container
   * @param AccelData Pointer to shared accelerometer data container
   * @param updateMs Sample interval in milliseconds (default 500ms = 2Hz)
   * 
   * @note updateMs determines telemetry rate; lower values increase CPU load
   * @note Recommended range: 100ms (10Hz) to 1000ms (1Hz)
   */
  IMUTask(Adafruit_BNO055* imu, Share<EulerAngles>* EulerAngles, Share<GyroData>* GyroData, 
    Share<AccelData>* AccelData, uint32_t updateMs = 500) noexcept;

  /**
   * @brief Initialize the BNO055 sensor hardware
   * 
   * Performs I2C communication test, sets NDOF mode, and verifies sensor presence.
   * Must be called before starting the FreeRTOS task.
   * 
   * @return true if sensor initialized successfully
   * @return false if sensor not found or communication failed
   * 
   * @note Prints diagnostic messages to Serial
   * @note Sets sensor to NDOF (9-DOF fusion) mode automatically
   */
  bool initIMU();

  /**
   * @brief Start the IMU FreeRTOS task
   * @param priority Task priority (1 = low, higher = more important)
   * @param core CPU core to pin task to (-1 = no pinning, 0 or 1 for specific core)
   */
  void start(uint8_t priority = 1, int8_t core = -1);
  
  /**
   * @brief Execute one FSM cycle - read sensor and update shares
   * @note Called repeatedly from FreeRTOS task loop
   */
  void update() noexcept;

  // -------------------------------------------------------------------------
  // Calibration Methods
  // -------------------------------------------------------------------------
  
  /**
   * @brief Print current calibration status to Serial
   * Displays system, gyro, accel, and mag calibration levels (0-3)
   */
  void printCalibrationStatus();
  
  /**
   * @brief Check if sensor is fully calibrated (all subsystems at level 3)
   * @return true if fully calibrated
   */
  bool isFullyCalibrated();
  
  /**
   * @brief Check if sensor data is reliable (system calibration >= 1)
   * @return true if magnetic north found and data trustworthy
   */
  bool isDataReliable();
  
  /**
   * @brief Read current calibration offsets from sensor
   * @param offsets Pointer to structure to receive offset data
   */
  void getCalibrationOffsets(adafruit_bno055_offsets_t* offsets);
  
  /**
   * @brief Write calibration offsets to sensor
   * @param offsets Pointer to structure containing offset data
   * @note Allows restoring saved calibration for faster startup
   */
  void setCalibrationOffsets(const adafruit_bno055_offsets_t* offsets);
  
  /**
   * @brief Print current calibration offsets to Serial
   * Useful for recording offsets to hardcode in firmware
   */
  void printCalibrationOffsets();
  
  /**
   * @brief Perform interactive calibration procedure
   * @param timeoutMs Maximum time to wait for full calibration (default 60 seconds)
   * @note Blocks until fully calibrated or timeout expires
   * @note Prints instructions to Serial for user
   */
  void performCalibration(uint32_t timeoutMs = 60000);
  
  /**
   * @brief Load hardcoded calibration offsets
   * Apply pre-saved calibration data from savedCalibrationData_ array
   */
  void loadSavedCalibration();

  // -------------------------------------------------------------------------
  // Configuration and Accessors
  // -------------------------------------------------------------------------
  
  /**
   * @brief Get current sample interval
   * @return Sample period in milliseconds
   */
  uint32_t get_updateMs() const noexcept { return updateMs_; }
  
  /**
   * @brief Set static instance pointer for FSM callbacks
   * @param inst Pointer to IMUTask instance
   */
  static void set_instance(IMUTask* inst) { instance_ = inst; }

  /**
   * @brief Change sample interval at runtime
   * @param intervalMs New sample period in milliseconds
   * @note Takes effect on next sample
   */
  void setSampleInterval(uint32_t intervalMs) { updateMs_ = intervalMs; }

private:
  // -------------------------------------------------------------------------
  // Timing Configuration
  // -------------------------------------------------------------------------
  
  uint32_t updateMs_ = 1000;      ///< Sample interval in milliseconds
  uint32_t lastSampleMs_ = 0;     ///< Timestamp of last sample (millis())

  // -------------------------------------------------------------------------
  // Hardware Interface
  // -------------------------------------------------------------------------
  
  Adafruit_BNO055* imu_;          ///< BNO055 sensor object pointer
  bool imuInitialized_ = false;   ///< True if initIMU() succeeded
  
  // -------------------------------------------------------------------------
  // Shared Data Containers
  // -------------------------------------------------------------------------
  
  Share<EulerAngles>* eulerAngles_;  ///< Euler angle output share
  Share<GyroData>* gyroData_;        ///< Gyroscope output share
  Share<AccelData>* accelData_;      ///< Accelerometer output share

  // -------------------------------------------------------------------------
  // Calibration Persistence
  // -------------------------------------------------------------------------
  
  uint8_t savedCalibrationData_[22];  ///< BNO055 offset registers (22 bytes)

  // -------------------------------------------------------------------------
  // FSM State Objects
  // -------------------------------------------------------------------------
  
  State state_wait_{0, "WAIT", &IMUTask::exec_wait};              ///< WAIT state object
  State state_send_data_{1, "SEND_DATA", &IMUTask::exec_sendData}; ///< SEND_DATA state object
  State* states_[2] = { &state_wait_, &state_send_data_ };       ///< State array for FSM
  FSM fsm_;  ///< Finite state machine manager

  // -------------------------------------------------------------------------
  // Static State Functions (FSM Callbacks)
  // -------------------------------------------------------------------------
  
  /**
   * @brief WAIT state executor - waits for first sample interval
   * @return Next state ID (transitions to SEND_DATA)
   */
  static uint8_t exec_wait();
  
  /**
   * @brief SEND_DATA state executor - continuous sensor reading
   * @return Next state ID (returns SEND_DATA to stay in state)
   */
  static uint8_t exec_sendData();

  // -------------------------------------------------------------------------
  // Static Instance Pointer
  // -------------------------------------------------------------------------
  
  static IMUTask* instance_;  ///< Singleton for static callback access
};
