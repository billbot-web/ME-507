#include <Arduino.h>
#include "UITask.h"
#include "EncoderTask.h"  // for EncoderTask::Command enum

// Static instance
UITask* UITask::instance_ = nullptr;

  UITask::UITask(Share<float>* positionShare,
               Share<float>*   velocityShare,
               Share<int8_t>* vref,
               Share<int16_t>* posref,
               Share<int8_t>*   cmdShare,
               uint32_t        updateMs) noexcept
  : positionShare_(positionShare),
    velocityShare_(velocityShare),
    vref_(vref),
    posref_(posref),
    cmdShare_(cmdShare),
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


