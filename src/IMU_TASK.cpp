#include <Arduino.h>
#include "IMU_TASK.h"
#include "UITask.h" // For IMU data structures

// Static instance
IMUTask* IMUTask::instance_ = nullptr;

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

// FreeRTOS C-style task entry function. Matches UITask pattern.
extern "C" void imu_task_func(void* pvParameters) {
  IMUTask* imu_task = static_cast<IMUTask*>(pvParameters);
  IMUTask::set_instance(imu_task);
  const TickType_t tick = pdMS_TO_TICKS(imu_task ? imu_task->get_updateMs() : 1000);
  for (;;) {
    if (imu_task) imu_task->update();
    vTaskDelay(tick);
  }
}

bool IMUTask::initIMU() {
  Serial.println("IMU task init...");
  
  if (!imu_) {
    Serial.println("Error: IMU pointer is null");
    return false;
  }
  
  // Initialize IMU; log but continue even if begin() fails so FSM can run
  if (!imu_->begin(OPERATION_MODE_NDOF)) {
    Serial.println("Warning: BNO055 not detected or failed to initialize.");
    return false;
  } else {
    Serial.println("BNO055 initialized.");
    return true;
  }
}


// Main update method called by the FreeRTOS task
void IMUTask::update() noexcept {
  // Run the finite state machine
  fsm_.run_curstate();
}

// ---------------- State implementations ----------------

uint8_t IMUTask::exec_wait() {
  if (!instance_) return static_cast<uint8_t>(WAIT);
  
  uint32_t now = millis();
  if ((now - instance_->lastSampleMs_) >= instance_->updateMs_) {
    // Time to send data
    return static_cast<uint8_t>(SEND_DATA);
  }
  return static_cast<uint8_t>(WAIT); // stay in WAIT
}

uint8_t IMUTask::exec_sendData() {
  if (!instance_ || !instance_->imu_) return static_cast<uint8_t>(WAIT);
  
  // Check calibration status before interpreting data
  uint8_t system, gyroCal, accelCal, magCal;
  instance_->imu_->getCalibration(&system, &gyroCal, &accelCal, &magCal);
  
  // In NDOF mode, discard data if system calibration is 0 
  // (device hasn't found magnetic north yet)
  if (system == 0) {
    return static_cast<uint8_t>(SEND_DATA);
  }
  // System calibration >= 1, data is reliable - read and output
  //Get the sensor vectors
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
