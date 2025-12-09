/**
 * @file ControllerTask.cpp
 * @brief Implementation of top-level FSM controller for LED tracking and manual control
 * 
 * This file implements a sophisticated multi-mode controller that coordinates camera-based
 * LED tracking, manual teleoperation, motor testing, and calibration. The task acts as the
 * system supervisor, managing state transitions and motor command generation.
 * 
 * State Machine Details:
 * - **CALIBRATE**: Homes motors to known positions using encoder feedback
 * - **WAIT**: Idle state with motors stopped
 * - **SCAN**: Sweeping search pattern when LED not detected
 * - **TRACKR**: Active tracking using camera pixel errors for PID control
 * - **TELEOP**: Manual D-pad control from web UI
 * - **MOTOR_TEST**: Open-loop testing for diagnostics
 * 
 * Tracking Behavior:
 * - Uses camera pixel errors (pan_err, tilt_err) from LED detection
 * - LED loss handled with 5-frame persistence counter before switching to SCAN
 * - Automatic recovery to TRACKR when LED redetected during SCAN
 * - Position limits enforced on tilt axis for mechanical safety
 * 
 * Motor Command Generation:
 * - Sets motor modes: 0=WAIT, 1=VRUN, 2=PRUN, 3=CAMERA_PRUN
 * - Updates velocity setpoints for VRUN and TELEOP modes
 * - Updates camera errors for CAMERA_PRUN tracking mode
 * 
 * @author Control Systems Team
 * @date November 2025
 * @version 2.1
 * 
 * @see ControllerTask.h for class interface and mode descriptions
 * @see MotorTask.h for motor control implementation details
 */

#include <Arduino.h>
#include "ControllerTask.h"
#include "taskshare.h"
#include "PID.h"

/// @brief Static instance pointer for FSM callback access
ControllerTask* ControllerTask::instance_ = nullptr;

/**
 * @brief Construct controller task with all necessary data shares
 * 
 * Initializes the controller task with pointers to all shared data containers
 * used for inter-task communication. Sets up FSM with all six states and
 * initializes LED loss tracking.
 * 
 * @param pan_err Camera pan error share (pixels from image center)
 * @param tilt_err Camera tilt error share (pixels from image center)
 * @param tiltpos Current tilt position share (degrees from encoder)
 * @param tiltVelo Tilt velocity command share (-100 to 100)
 * @param panVelo Pan velocity command share (-100 to 100)
 * @param tilt_mode Tilt motor mode command (0=WAIT, 1=VRUN, 2=PRUN, 3=CAMERA_PRUN)
 * @param pan_mode Pan motor mode command (0=WAIT, 1=VRUN, 2=PRUN, 3=CAMERA_PRUN)
 * @param Cam_mode Camera mode control share
 * @param hasLed LED detection status share (true when LED visible)
 * @param UI_mode User interface mode selection share
 * @param dcalibrate Calibration trigger share (true to start calibration)
 * @param dpad_pan D-pad pan direction (-1=left, 0=none, 1=right)
 * @param dpad_tilt D-pad tilt direction (-1=down, 0=none, 1=up)
 * @param updateMs FSM update period in milliseconds (default 100ms = 10Hz)
 */
ControllerTask::ControllerTask(Share<int16_t>* pan_err,
                                       Share<int16_t>*  tilt_err,
                                       Share<float_t>*  tiltpos,
                                       Share<int8_t>*  tiltVelo,
                                       Share<int8_t>*  panVelo,
                                       Share<uint8_t>* tilt_mode,
                                       Share<uint8_t>* pan_mode,
                                       Share<uint8_t>* Cam_mode,
                                       Share<bool>*       hasLed,
                                       Share<uint8_t>*   UI_mode,
                                       Share<bool>*   dcalibrate,
                                       Share<int8_t>*  dpad_pan,
                                       Share<int8_t>*  dpad_tilt,
                                       uint32_t updateMs) noexcept
  : updateMs_(updateMs),
    pan_err_(pan_err),
    tilt_err_(tilt_err),
    tiltpos_(tiltpos),
    tiltVelo_(tiltVelo),
    panVelo_(panVelo),
    tilt_mode_(tilt_mode),
    pan_mode_(pan_mode),
    Cam_mode_(Cam_mode),
    hasLed_(hasLed),
    UI_mode_(UI_mode),
    dcalibrate_(dcalibrate),
    dpad_pan_(dpad_pan),
    dpad_tilt_(dpad_tilt),
    led_loss_counter_(0),
    //fsm initialization
    fsm_(states_, 6)
{
    instance_ = this;
    // Initialize output to zero effort to both motors
    if (panVelo_) panVelo_->put(0);
    if (tiltVelo_) tiltVelo_->put(0);
}

/**
 * @brief FreeRTOS task entry; repeatedly calls update() at configured period.
 */
extern "C" void controller_task_func(void* arg) {
    ControllerTask* ctl = static_cast<ControllerTask*>(arg);
    const TickType_t delayTicks = pdMS_TO_TICKS(ctl->getUpdateMs()); // 100 Hz default
    for (;;) {
        if (ctl) ctl->update();
        vTaskDelay(delayTicks);
    }
}
// ---------------------------------------------------------------------------
// STATE: CTL_WAIT
// Effort forced to 0, wait for RUN/ZERO commands.
// ---------------------------------------------------------------------------
uint8_t ControllerTask::exec_calibrate()
{
    if (!instance_) return CALIBRATE;

    // Ensure effort is zero in WAIT
    if (instance_->panVelo_) instance_->panVelo_->put(0);
    if (instance_->tiltVelo_) instance_->tiltVelo_->put(0);
    
    //implement calibration routine here later
    // Check for dcalibrate command if done calibrating go to wait
    if (instance_->dcalibrate_) {
        bool dcalibrate = instance_->dcalibrate_->get();
        if (dcalibrate) {
            return WAIT;
        }
    }
    return CALIBRATE;
}
uint8_t ControllerTask::exec_wait()
{
    if (!instance_) return WAIT;

    //instance_->ConstrainTiltMotor(instance_->tiltpos_);

    // Read latest command form UI (non-blocking)
    int8_t cmd = 0;
    if (instance_->UI_mode_) {
        cmd = instance_->UI_mode_->get();
    }

    static uint32_t lastDebug = 0;
    if (millis() - lastDebug > 1000) {
        Serial.print("[ControllerTask] WAIT state - UI_mode value: ");
        Serial.println(cmd);
        lastDebug = millis();
    }

    switch (cmd) {
        case 3: // motor test mode
            Serial.println("Switching to MOTOR_TEST mode from wait");
            return MOTOR_TEST;

        case 2: // run in telop mode
            Serial.println("Switching to TELEOP mode from wait");
            if (instance_-> pan_mode_) instance_->pan_mode_->put(1);
            if (instance_-> tilt_mode_) instance_->tilt_mode_->put(1);
            return TELEOP;

        case 1: //run in scan mode
            Serial.println("Switching to SCAN mode from wait");
            // set scan velocities
            if (instance_-> pan_mode_) instance_->pan_mode_->put(1);
            if (instance_-> tilt_mode_) instance_->tilt_mode_->put(1);
            if (instance_->panVelo_) instance_->panVelo_->put(instance_->SCAN_PAN_VELO);
            if (instance_->tiltVelo_) instance_->tiltVelo_->put(instance_->SCAN_PAN_VELO);
            if (instance_ ->Cam_mode_) instance_->Cam_mode_->put(1); //set camera to tracking mode
            return SCAN;

        case 0:
        default:
            break;
    }

    return WAIT;
}
// ---------------------------------------------------------------------------
// STATE: exec_scan
// Scan for target, track when found.
// ---------------------------------------------------------------------------
uint8_t ControllerTask::exec_scan()
{
    if (!instance_) return WAIT;

    // Handle commands while running
    int8_t cmd = 1;
    if (instance_->UI_mode_) {
        cmd = instance_->UI_mode_->get();
    }
    
    switch (cmd) {
        case 0: // stop go back to wait
            //set mototo modes
            // set motor efforts to zero
            if (instance_->pan_mode_) instance_->pan_mode_->put(0);
            if (instance_->tilt_mode_) instance_->tilt_mode_->put(0);
            if (instance_->panVelo_) instance_->panVelo_->put(0);
            if (instance_->tiltVelo_) instance_->tiltVelo_->put(0);
            if (instance_-> Cam_mode_) instance_->Cam_mode_->put(0); //set camera to idle mode
            return WAIT;

        case 2: //go to teleop mode
            Serial.println("Switching to TELEOP mode from SCAN");
            if (instance_->panVelo_) instance_->panVelo_->put(0);
            if (instance_->tiltVelo_) instance_->tiltVelo_->put(0);
            if (instance_-> Cam_mode_) instance_->Cam_mode_->put(0); //set camera to idle mode
            //set motor modes to 0
            if (instance_->pan_mode_) instance_->pan_mode_->put(1);
            if (instance_->tilt_mode_) instance_->tilt_mode_->put(1);
             //set motor modes
            return TELEOP;
        case 3: //go to motor test mode
            Serial.println("Switching to MOTOR_TEST mode from SCAN");
            if (instance_->panVelo_) instance_->panVelo_->put(0);
            if (instance_->tiltVelo_) instance_->tiltVelo_->put(0);
            if (instance_-> Cam_mode_) instance_->Cam_mode_->put(0); //set camera to idle mode
            return MOTOR_TEST;
    }
    // if tilt motor is at limit reverse direction
    if (instance_->tiltpos_) {
        float tilt_position = instance_->tiltpos_->get();
        if (tilt_position >= 10.0f) { // upper limit
            if (instance_->tiltVelo_) {
            instance_->tiltVelo_->put(-instance_->SCAN_PAN_VELO);
            if (instance_->tilt_mode_) instance_->tilt_mode_->put(1);
            }
        } else if (tilt_position <= -150.0f) { // lower limit
            if (instance_->tiltVelo_) instance_->tiltVelo_->put(instance_->SCAN_PAN_VELO);
            if (instance_->tilt_mode_) instance_->tilt_mode_->put(1);
        }
    }
    // Check if LED is found to switch to TRACKR
    if (instance_->hasLed_) {
        bool led_found = instance_->hasLed_->get();
        if (led_found) {
            if (instance_-> pan_mode_) instance_->pan_mode_->put(3);
            if (instance_-> tilt_mode_) instance_->tilt_mode_->put(3);
            if (instance_->panVelo_) instance_->panVelo_->put(0);
            if (instance_->tiltVelo_) instance_->tiltVelo_->put(0);
            return TRACKR;
        }
    }
    // Continue scanning if no led was found
    return SCAN;  
}

uint8_t ControllerTask::exec_trackr()
{
    if (!instance_) return WAIT;



    // Handle commands while running
    int8_t cmd = 1;
    if (instance_->UI_mode_) {
        cmd = instance_->UI_mode_->get();
    }

    switch (cmd) {
        case 0: // STOP
            // set motor efforts to zero
            if (instance_->panVelo_) instance_->panVelo_->put(0);
            if (instance_->tiltVelo_) instance_->tiltVelo_->put(0);
            //set motor modes to 0
            if (instance_->pan_mode_) instance_->pan_mode_->put(0);
            if (instance_->tilt_mode_) instance_->tilt_mode_->put(0);
            if (instance_-> Cam_mode_) instance_->Cam_mode_->put(0); //set camera to idle mode
            return WAIT;

        case 3: 
            //go to teleop mode
            Serial.println("Switching to Motor Test mode from TRACKR");
            if (instance_->panVelo_) instance_->panVelo_->put(0); //set motor modes to 0
            if (instance_->tiltVelo_) instance_->tiltVelo_->put(0); //set motor modes to 0
            if (instance_-> Cam_mode_) instance_->Cam_mode_->put(0); //set cmamera to idle mode
            return MOTOR_TEST;


        case 2: // ZERO integrator but keep running
            if (instance_->panVelo_) instance_->panVelo_->put(0);
            if (instance_->tiltVelo_) instance_->tiltVelo_->put(0);
            //set motor modes to 0
            if (instance_->pan_mode_) instance_->pan_mode_->put(1);
            if (instance_->tilt_mode_) instance_->tilt_mode_->put(1);
            if (instance_-> Cam_mode_) instance_->Cam_mode_->put(0); //set camera to idle mode
            return TELEOP;

        case 1:
        default:
            break;
    }
    // Check if LED was lost to switch to SCAN
    if (instance_->hasLed_) {
        bool led_found = instance_->hasLed_->get();
        if (!led_found) {
            instance_->led_loss_counter_++;
            if (instance_->led_loss_counter_ >= instance_->LED_LOSS_THRESHOLD) {
                // Lost LED for 5 consecutive cycles, switch to SCAN
                Serial.println("LED lost for 5 cycles, switching to SCAN");
                if (instance_->panVelo_) instance_->panVelo_->put(instance_->SCAN_PAN_VELO);
                if (instance_->tiltVelo_) instance_->tiltVelo_->put(instance_->SCAN_PAN_VELO);
                if (instance_-> pan_mode_) instance_->pan_mode_->put(1);
                if (instance_-> tilt_mode_) instance_->tilt_mode_->put(1);
                if (instance_-> Cam_mode_) instance_->Cam_mode_->put(1);
                instance_->led_loss_counter_ = 0;  // Reset counter
                return SCAN;
            }
            // LED lost but not enough cycles yet, stay in TRACKR
        } else {
            // LED found, reset counter
            instance_->led_loss_counter_ = 0;
        }
    }
    // if tilt motor is at limit stop motors and go to wait
    if (instance_->tiltpos_) {
        float tilt_position = instance_->tiltpos_->get();
        if (tilt_position >= 30.0f || tilt_position <= -180.0f) { // upper/lower limit
            if (instance_->panVelo_) instance_->panVelo_->put(0);
            if (instance_->tiltVelo_) instance_->tiltVelo_->put(0);
            if (instance_->pan_mode_) instance_->pan_mode_->put(0);
            if (instance_->tilt_mode_) instance_->tilt_mode_->put(0);
            Serial.println("Object out of range, Tilt motor at limit, returning to WAIT");
            return WAIT;
        }
    }
    
    // Continue tracking if LED is still in frame
    return TRACKR;  
}

uint8_t ControllerTask::exec_teleop()
{
    if (!instance_) return WAIT;

    

    // Handle commands while running
    int8_t cmd = 0;
    if (instance_->UI_mode_) {
        cmd = instance_->UI_mode_->get();
    }
    // Read dpad inputs
    if (instance_->dpad_pan_ && instance_->dpad_tilt_) {
        int8_t pan_dir = instance_->dpad_pan_->get();
        int8_t tilt_dir = instance_->dpad_tilt_->get();

        // Set velocities based on dpad input
        if (instance_->panVelo_) instance_->panVelo_->put(pan_dir * instance_->TELEOP_VELO); 
        // 30 deg/sec per button press
        if (instance_->tiltVelo_) instance_->tiltVelo_->put(tilt_dir * instance_->TELEOP_VELO);
    }

    switch (cmd) {
        case 0: // STOP
            // set motor efforts to zero
            if (instance_->panVelo_) instance_->panVelo_->put(0);
            if (instance_->tiltVelo_) instance_->tiltVelo_->put(0);
            //set motor modes to 0
            if (instance_->pan_mode_) instance_->pan_mode_->put(0);
            if (instance_->tilt_mode_) instance_->tilt_mode_->put(0);
            return WAIT;

        case 1:
            //set scan velocities
            if (instance_->pan_mode_) instance_->pan_mode_->put(1);
            if (instance_->tilt_mode_) instance_->tilt_mode_->put(1);
            if (instance_->panVelo_) instance_->panVelo_->put(instance_->SCAN_PAN_VELO);
            if (instance_->tiltVelo_) instance_->tiltVelo_->put(instance_->SCAN_PAN_VELO);
            if (instance_-> Cam_mode_) instance_->Cam_mode_->put(1); //set camera to idle mode
            return SCAN;
        case 3: //go to motor test mode
            if (instance_->panVelo_) instance_->panVelo_->put(0);
            if (instance_->tiltVelo_) instance_->tiltVelo_->put(0);
            //set motor modes to 0
            if (instance_->pan_mode_) instance_->pan_mode_->put(1);
            if (instance_->tilt_mode_) instance_->tilt_mode_->put(1);
            return MOTOR_TEST;
    }

    // --- PID computation ---

    return TELEOP;   // stay in RUN until STOP command
}

uint8_t ControllerTask::exec_motor_test()
{
    if (!instance_) return WAIT;

    // Handle commands while running
    int8_t cmd = 3;
    if (instance_->UI_mode_) {
        cmd = instance_->UI_mode_->get();
    }

    // In motor test mode, the UI directly controls motor velocities
    // via tiltVelo_ and panVelo_ shares, so we don't need to do anything here
    // except monitor for mode changes

    switch (cmd) {
        case 0: // STOP - return to wait
            // set motor efforts to zero
            if (instance_->panVelo_) instance_->panVelo_->put(0);
            if (instance_->tiltVelo_) instance_->tiltVelo_->put(0);
            //set motor modes to 0
            if (instance_->pan_mode_) instance_->pan_mode_->put(0);
            if (instance_->tilt_mode_) instance_->tilt_mode_->put(0);
            Serial.println("[ControllerTask] Returning to WAIT from MOTOR_TEST");
            return WAIT;

        case 1: // Switch to SCAN
            Serial.println("[ControllerTask] Switching to SCAN from MOTOR_TEST");
            if (instance_->pan_mode_) instance_->pan_mode_->put(1);
            if (instance_->tilt_mode_) instance_->tilt_mode_->put(1);
            if (instance_->panVelo_) instance_->panVelo_->put(instance_->SCAN_PAN_VELO);
            if (instance_->tiltVelo_) instance_->tiltVelo_->put(instance_->SCAN_PAN_VELO);
            return SCAN;
        case 2: // Switch to TELEOP
            if (instance_->pan_mode_) instance_->pan_mode_->put(1);
            if (instance_->tilt_mode_) instance_->tilt_mode_->put(1);
            Serial.println("[ControllerTask] Switching to TELEOP from MOTOR_TEST");
            return TELEOP;

        case 3: // Stay in MOTOR_TEST
        default:
            break;
    }

    return MOTOR_TEST;
}

