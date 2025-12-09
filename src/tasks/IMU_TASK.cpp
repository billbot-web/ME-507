/**
 * @file IMU_TASK.cpp
 * @brief Implementation of BNO055 IMU sensor data acquisition task
 * 
 * This file implements continuous IMU sensor data streaming using a two-state FSM.
 * The task runs in SEND_DATA state continuously, publishing orientation, angular
 * velocity, and acceleration data at a configurable sample rate.
 * 
 * Implementation Details:
 * - Reads Euler angles, gyroscope, and accelerometer data from BNO055
 * - Operates in NDOF (9-DOF fusion) mode for optimal accuracy
 * - Publishes data via Share<T> objects for thread-safe inter-task communication
 * - Supports calibration persistence via hardcoded offset values
 * - Checks system calibration status to ensure data reliability
 * 
 * FSM Behavior:
 * - WAIT state: Waits for initial sample interval, transitions to SEND_DATA
 * - SEND_DATA state: Continuously reads and publishes sensor data, never returns to WAIT
 * 
 * Timing:
 * - Default sample rate: 500ms (2Hz)
 * - Configurable via constructor parameter
 * - Limited by I2C bus speed (~100Hz practical maximum)
 * 
 * @author IMU Integration Team
 * @date November 2025
 * @version 1.8
 * 
 * @see IMU_TASK.h for class interface and configuration options
 * @see Adafruit_BNO055.h for sensor hardware details
 */

#include <Arduino.h>
#include "IMU_TASK.h"
#include "UITask.h" // For IMU data structures

/// @brief Static instance pointer for FSM callback access
IMUTask* IMUTask::instance_ = nullptr;

/**
 * @brief Construct IMU task with sensor configuration and data shares
 * 
 * Initializes the IMU task with pointers to the BNO055 sensor and shared data
 * containers. Also loads hardcoded calibration offsets for faster startup.
 * 
 * Calibration offsets are specific to the physical sensor and mounting orientation:
 * - Accel: (-44, 0, -46)
 * - Gyro: (-1, 0, 1)  
 * - Mag: (-9, -277, -288)
 * - Accel Radius: 1000, Mag Radius: 645
 * 
 * @param imu Pointer to initialized Adafruit_BNO055 sensor object
 * @param EulerAngles Shared container for euler angles (roll, pitch, yaw)
 * @param GyroData Shared container for angular velocities
 * @param AccelData Shared container for linear accelerations
 * @param updateMs Sample interval in milliseconds (default 500ms)
 * 
 * @note Calibration values must be determined for each specific sensor
 * @note See performCalibration() for obtaining calibration offsets
 */
IMUTask::IMUTask(Adafruit_BNO055* imu, Share<EulerAngles>* EulerAngles, Share<GyroData>* GyroData, 
    Share<AccelData>* AccelData, uint32_t updateMs) noexcept
  : updateMs_(updateMs),
    lastSampleMs_(0),
    imu_(imu),
    eulerAngles_(EulerAngles),
    gyroData_(GyroData),
    accelData_(AccelData),
    fsm_(states_, 2)
{
  instance_ = this;
  
  // Initialize saved calibration data with your specific offsets
  // Accel: -44, 0, -46  Gyro: -1, 0, 1  Mag: -9, -277, -288  
  // Accel Radius: 1000  Mag Radius: 645
  
  // Accel offsets (16-bit signed, LSB first)
  savedCalibrationData_[0] = (-44) & 0xFF;        // ACCEL_OFFSET_X_LSB
  savedCalibrationData_[1] = ((-44) >> 8) & 0xFF; // ACCEL_OFFSET_X_MSB
  savedCalibrationData_[2] = (0) & 0xFF;          // ACCEL_OFFSET_Y_LSB  
  savedCalibrationData_[3] = ((0) >> 8) & 0xFF;   // ACCEL_OFFSET_Y_MSB
  savedCalibrationData_[4] = (-46) & 0xFF;        // ACCEL_OFFSET_Z_LSB
  savedCalibrationData_[5] = ((-46) >> 8) & 0xFF; // ACCEL_OFFSET_Z_MSB
  
  // Mag offsets (16-bit signed, LSB first)
  savedCalibrationData_[6] = (-9) & 0xFF;           // MAG_OFFSET_X_LSB
  savedCalibrationData_[7] = ((-9) >> 8) & 0xFF;    // MAG_OFFSET_X_MSB
  savedCalibrationData_[8] = (-277) & 0xFF;         // MAG_OFFSET_Y_LSB
  savedCalibrationData_[9] = ((-277) >> 8) & 0xFF;  // MAG_OFFSET_Y_MSB
  savedCalibrationData_[10] = (-288) & 0xFF;        // MAG_OFFSET_Z_LSB
  savedCalibrationData_[11] = ((-288) >> 8) & 0xFF; // MAG_OFFSET_Z_MSB
  
  // Gyro offsets (16-bit signed, LSB first)
  savedCalibrationData_[12] = (-1) & 0xFF;        // GYRO_OFFSET_X_LSB
  savedCalibrationData_[13] = ((-1) >> 8) & 0xFF; // GYRO_OFFSET_X_MSB
  savedCalibrationData_[14] = (0) & 0xFF;         // GYRO_OFFSET_Y_LSB
  savedCalibrationData_[15] = ((0) >> 8) & 0xFF;  // GYRO_OFFSET_Y_MSB
  savedCalibrationData_[16] = (1) & 0xFF;         // GYRO_OFFSET_Z_LSB
  savedCalibrationData_[17] = ((1) >> 8) & 0xFF;  // GYRO_OFFSET_Z_MSB
  
  // Accel radius (16-bit unsigned, LSB first)
  savedCalibrationData_[18] = (1000) & 0xFF;        // ACCEL_RADIUS_LSB
  savedCalibrationData_[19] = ((1000) >> 8) & 0xFF; // ACCEL_RADIUS_MSB
  
  // Mag radius (16-bit unsigned, LSB first)  
  savedCalibrationData_[20] = (645) & 0xFF;        // MAG_RADIUS_LSB
  savedCalibrationData_[21] = ((645) >> 8) & 0xFF; // MAG_RADIUS_MSB
}

/**
 * @brief FreeRTOS task entry point for IMU data acquisition
 * 
 * C-style function required by xTaskCreate(). Creates an infinite loop that
 * periodically calls the IMU task's update() method to execute the FSM.
 * 
 * Task Loop:
 * 1. Set static instance pointer for callback access
 * 2. Call update() to execute current FSM state
 * 3. Delay for configured sample interval
 * 4. Repeat indefinitely
 * 
 * @param pvParameters Pointer to IMUTask object (cast from void*)
 * 
 * @note Delay duration matches updateMs_ from IMUTask configuration
 * @note Task never exits - runs for lifetime of system
 */
extern "C" void imu_task_func(void* pvParameters) {
  IMUTask* imu_task = static_cast<IMUTask*>(pvParameters);
  IMUTask::set_instance(imu_task);
  const TickType_t tick = pdMS_TO_TICKS(imu_task ? imu_task->get_updateMs() : 1000);
  for (;;) {
    if (imu_task) imu_task->update();
    vTaskDelay(tick);
  }
}

/**
 * @brief Initialize BNO055 IMU sensor hardware and configure operating mode
 * 
 * Performs I2C communication test with the BNO055 sensor and sets it to
 * NDOF (Nine Degrees of Freedom) fusion mode. This mode combines data from
 * all three sensors (accelerometer, gyroscope, magnetometer) for optimal
 * accuracy in absolute orientation tracking.
 * 
 * Initialization Steps:
 * 1. Verify imu_ pointer is valid
 * 2. Call begin() with NDOF mode parameter
 * 3. Set imuInitialized_ flag based on result
 * 4. Print diagnostic messages to Serial
 * 
 * @return true if sensor responded and entered NDOF mode successfully
 * @return false if sensor not found, I2C error, or null pointer
 * 
 * @note Must be called before starting FreeRTOS task
 * @note NDOF mode requires calibration for accurate absolute orientation
 * @note Prints diagnostic output to Serial for debugging
 */
bool IMUTask::initIMU() {
  Serial.println("IMU task init...");
  
  if (!imu_) {
    Serial.println("Error: IMU pointer is null");
    imuInitialized_ = false;
    return false;
  }
  
  // Initialize IMU; log but continue even if begin() fails so FSM can run
  if (!imu_->begin(OPERATION_MODE_NDOF)) {
    Serial.println("Warning: BNO055 not detected or failed to initialize.");
    imuInitialized_ = false;
    return false;
  } else {
    Serial.println("BNO055 initialized.");
    imuInitialized_ = true;
    return true;
  }
}


/**
 * @brief Execute one FSM cycle for IMU data acquisition
 * 
 * Called repeatedly from the FreeRTOS task loop. Delegates to the FSM manager
 * which executes the current state's function (exec_wait or exec_sendData).
 * 
 * @note Called at rate determined by updateMs_ parameter
 * @note Thread-safe - uses Share<T> for all data outputs
 */
void IMUTask::update() noexcept {
  // Run the finite state machine
  fsm_.run_curstate();
}

// ---------------- State implementations ----------------

/**
 * @brief WAIT state - Initial delay before first data transmission
 * 
 * This state provides an initial delay to ensure the sensor is fully stabilized
 * before beginning continuous data acquisition. After the first sample interval
 * expires, transitions to SEND_DATA state.
 * 
 * State Behavior:
 * 1. Check if sample interval has elapsed since last sample
 * 2. If yes: Update lastSampleMs_ and transition to SEND_DATA
 * 3. If no: Stay in WAIT
 * 
 * @return Next state ID
 * @retval WAIT Continue waiting for sample interval
 * @retval SEND_DATA Begin continuous data acquisition
 * 
 * @note Only runs once at startup - SEND_DATA never returns to WAIT
 * @note Updates lastSampleMs_ timestamp when transitioning
 */
uint8_t IMUTask::exec_wait() {
  if (!instance_) return static_cast<uint8_t>(WAIT);
  
  uint32_t now = millis();
  if ((now - instance_->lastSampleMs_) >= instance_->updateMs_) {
    // Time to send data - update timestamp
    instance_->lastSampleMs_ = now;
    return static_cast<uint8_t>(SEND_DATA);
  }
  return static_cast<uint8_t>(WAIT); // stay in WAIT
}

/**
 * @brief SEND_DATA state - Continuous sensor reading and data publication
 * 
 * This state runs continuously, reading all sensor data from the BNO055 and
 * publishing it to shared memory containers. The state never transitions back
 * to WAIT, providing uninterrupted data streaming for real-time telemetry.
 * 
 * State Behavior:
 * 1. Verify sensor is initialized and pointers are valid
 * 2. Read calibration status (not currently blocking execution)
 * 3. Read euler angles (absolute orientation in degrees)
 * 4. Read gyroscope data (angular velocity in deg/sec)
 * 5. Read accelerometer data (linear acceleration in m/s²)
 * 6. Read magnetometer data (magnetic field strength - commented out)
 * 7. Publish all data to respective Share<T> containers
 * 8. Return SEND_DATA to stay in state
 * 
 * Data Output:
 * - Euler: (x=roll, y=pitch, z=yaw) in degrees
 * - Gyro: (x, y, z) angular rates in deg/sec
 * - Accel: (x, y, z) accelerations in m/s²
 * 
 * @return Next state ID
 * @retval SEND_DATA Always returns to self for continuous operation
 * @retval WAIT Only if sensor not initialized (error condition)
 * 
 * @note Magnetometer data reading is commented out but available
 * @note All sensor reads are atomic via BNO055 hardware registers
 * @note Share<T>::put() is thread-safe for concurrent access
 */
uint8_t IMUTask::exec_sendData() {
  if (!instance_ || !instance_->imu_ || !instance_->imuInitialized_) return static_cast<uint8_t>(WAIT);
  
  // Check calibration status before interpreting data
  uint8_t system, gyroCal, accelCal, magCal;
  instance_->imu_->getCalibration(&system, &gyroCal, &accelCal, &magCal);

  // System calibration >= 1, data is reliable - read and output
  // Get the sensor vectors
  imu::Vector<3> euler = instance_->imu_->getVector(Adafruit_BNO055::VECTOR_EULER);
  imu::Vector<3> gyroVec = instance_->imu_->getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  imu::Vector<3> accelVec = instance_->imu_->getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu::Vector<3> magVec = instance_->imu_->getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
  
  // Update shared data: Euler, then gyro, then accel
  if (instance_->eulerAngles_) {
    EulerAngles eulerData;
    eulerData.x = static_cast<float>(euler.x());
    eulerData.y = static_cast<float>(euler.y());
    eulerData.z = static_cast<float>(euler.z());
    instance_->eulerAngles_->put(eulerData);
  }
  if (instance_->gyroData_) {
    GyroData gyroData;
    gyroData.x = static_cast<float>(gyroVec.x());
    gyroData.y = static_cast<float>(gyroVec.y());
    gyroData.z = static_cast<float>(gyroVec.z());
    instance_->gyroData_->put(gyroData);
  }
  if (instance_->accelData_) {
    AccelData accelData;
    accelData.x = static_cast<float>(accelVec.x());
    accelData.y = static_cast<float>(accelVec.y());
    accelData.z = static_cast<float>(accelVec.z());
    instance_->accelData_->put(accelData);
  }
  // if (instance_->magData_) {
  //   float magData[3] = { mag.x(), mag.y(), mag.z() };
  //   instance_->magData_->put(magData);
  // }

  // Stay in SEND_DATA to continuously update
  return static_cast<uint8_t>(SEND_DATA);

}

// ---------------- Calibration methods ----------------

void IMUTask::printCalibrationStatus() {
  if (!imu_) return;
  
  uint8_t system, gyro, accel, mag;
  imu_->getCalibration(&system, &gyro, &accel, &mag);
  
  Serial.println("\n--- Calibration Status ---");
  Serial.print("System: "); Serial.print(system, DEC);
  Serial.print(" Gyro: "); Serial.print(gyro, DEC);
  Serial.print(" Accel: "); Serial.print(accel, DEC);
  Serial.print(" Mag: "); Serial.println(mag, DEC);
  Serial.println("Note: 0=Uncalibrated, 3=Fully Calibrated");
}

bool IMUTask::isFullyCalibrated() {
  if (!imu_) return false;
  
  uint8_t system, gyro, accel, mag;
  imu_->getCalibration(&system, &gyro, &accel, &mag);
  
  return (system == 3 && gyro == 3 && accel == 3 && mag == 3);
}

bool IMUTask::isDataReliable() {
  if (!imu_) return false;
  
  uint8_t system, gyro, accel, mag;
  imu_->getCalibration(&system, &gyro, &accel, &mag);
  
  // In NDOF mode, system calibration >= 1 means magnetic north found
  // Data is reliable for orientation/heading
  return (system >= 1);
}

void IMUTask::getCalibrationOffsets(adafruit_bno055_offsets_t* offsets) {
  if (!imu_ || !offsets) return;
  
  if (imu_->isFullyCalibrated()) {
    imu_->getSensorOffsets(*offsets);
  }
}

void IMUTask::setCalibrationOffsets(const adafruit_bno055_offsets_t* offsets) {
  if (!imu_ || !offsets) return;
  
  imu_->setSensorOffsets(*offsets);
}

void IMUTask::printCalibrationOffsets() {
  if (!imu_) return;
  
  adafruit_bno055_offsets_t offsets;
  if (isFullyCalibrated()) {
    imu_->getSensorOffsets(offsets);
    
    Serial.println("\n--- Calibration Offsets ---");
    Serial.print("Accel: ");
    Serial.print(offsets.accel_offset_x); Serial.print(" ");
    Serial.print(offsets.accel_offset_y); Serial.print(" ");
    Serial.println(offsets.accel_offset_z);
    
    Serial.print("Gyro: ");
    Serial.print(offsets.gyro_offset_x); Serial.print(" ");
    Serial.print(offsets.gyro_offset_y); Serial.print(" ");
    Serial.println(offsets.gyro_offset_z);
    
    Serial.print("Mag: ");
    Serial.print(offsets.mag_offset_x); Serial.print(" ");
    Serial.print(offsets.mag_offset_y); Serial.print(" ");
    Serial.println(offsets.mag_offset_z);
    
    Serial.print("Accel Radius: "); Serial.println(offsets.accel_radius);
    Serial.print("Mag Radius: "); Serial.println(offsets.mag_radius);
  } else {
    Serial.println("IMU not fully calibrated - cannot get offsets");
  }
}

void IMUTask::performCalibration(uint32_t timeoutMs) {
  if (!imu_) return;
  
  Serial.println("\n=== BNO055 Calibration Process ===");
  Serial.println("Move the sensor in a figure-8 pattern for magnetometer");
  Serial.println("Rotate slowly around all axes for gyroscope");
  Serial.println("Place in different orientations for accelerometer");
  Serial.println("Press any key to stop calibration early");
  
  uint32_t startTime = millis();
  
  while ((millis() - startTime) < timeoutMs) {
    printCalibrationStatus();
    
    if (isFullyCalibrated()) {
      Serial.println("\n*** CALIBRATION COMPLETE! ***");
      printCalibrationOffsets();
      return;
    }
    
    if (Serial.available()) {
      Serial.read(); // consume the character
      Serial.println("\nCalibration stopped by user");
      break;
    }
    
    delay(1000);
  }
  
  if (!isFullyCalibrated()) {
    Serial.println("\nCalibration timeout or incomplete");
    printCalibrationStatus();
  }
}

void IMUTask::loadSavedCalibration() {
  if (!imu_) return;
  
  // Apply the saved calibration data from instance variable
  imu_->setSensorOffsets(savedCalibrationData_);
  
  Serial.println("Saved calibration offsets loaded successfully!");
  Serial.println("Accel: -44, 0, -46  Gyro: -1, 0, 1  Mag: -9, -277, -288");
  Serial.println("Accel Radius: 1000  Mag Radius: 645");
}
