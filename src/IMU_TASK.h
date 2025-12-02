/**
 * @file IMU_TASK.h
 * @brief FreeRTOS task for BNO055 IMU sensor management with FSM-based control
 * 
 * @author IMU Integration Team
 * @date November 2025
 * @version 1.7
 */

#pragma once
#include <Arduino.h>
#include <cstdint>
#include "FSM.h"
#include "State.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "Adafruit_BNO055.h"
#include "taskshare.h"

// IMU data structures
struct EulerAngles {
  float_t x, y, z;
};

struct GyroData {
  float_t x, y, z;
};

struct AccelData {
  float_t x, y, z;
};


/**
 * @brief IMU task with BNO055 sensor reading using FSM
 *
 * Features:
 * - Two-state FSM: WAIT and SEND_DATA
 * - Configurable sampling interval
 * - Serial output of IMU data (Euler angles)
 * - FreeRTOS task integration
 */
class IMUTask {
public:
  enum StateId : uint8_t {
    WAIT = 0,
    SEND_DATA = 1,
  };

  /**
   * @brief Construct IMU task.
   * @param imu Pointer to IMU sensor instance
   * @param EulerAngles Pointer to Euler angles share
   * @param GyroData Pointer to gyro data share  
   * @param AccelData Pointer to accel data share
   * @param updateMs IMU sampling interval in ms
   */
  IMUTask(Adafruit_BNO055* imu, Share<EulerAngles>* EulerAngles, Share<GyroData>* GyroData, 
    Share<AccelData>* AccelData, uint32_t updateMs = 500) noexcept;

  /**
   * @brief Initialize the IMU sensor
   * @return true if successful
   */
  bool initIMU();

  void start(uint8_t priority = 1, int8_t core = -1);
  void update() noexcept;

  // Calibration methods
  void printCalibrationStatus();
  bool isFullyCalibrated();
  bool isDataReliable(); // Check if system calibration >= 1 (found magnetic north)
  void getCalibrationOffsets(adafruit_bno055_offsets_t* offsets);
  void setCalibrationOffsets(const adafruit_bno055_offsets_t* offsets);
  void printCalibrationOffsets();
  void performCalibration(uint32_t timeoutMs = 60000);
  void loadSavedCalibration(); // Apply your specific calibration offsets

  // Public accessors used by the C-style task entry
  uint32_t get_updateMs() const noexcept { return updateMs_; }
  static void set_instance(IMUTask* inst) { instance_ = inst; }

  // Runtime configuration
  void setSampleInterval(uint32_t intervalMs) { updateMs_ = intervalMs; }

private:
  // Timing
  uint32_t updateMs_ = 1000;
  uint32_t lastSampleMs_ = 0;

  // IMU sensor
  Adafruit_BNO055* imu_;
  bool imuInitialized_ = false;
  
  // Shared data
  Share<EulerAngles>* eulerAngles_;
  Share<GyroData>* gyroData_;
  Share<AccelData>* accelData_;

  // Saved calibration data (22 bytes for BNO055 offset registers)
  uint8_t savedCalibrationData_[22];

  // FSM + states
  State state_wait_{0, "WAIT", &IMUTask::exec_wait};
  State state_send_data_{1, "SEND_DATA", &IMUTask::exec_sendData};
  State* states_[2] = { &state_wait_, &state_send_data_ };
  FSM fsm_;

  // State functions
  static uint8_t exec_wait();
  static uint8_t exec_sendData();

  // Singleton for static callbacks
  static IMUTask* instance_;
};
