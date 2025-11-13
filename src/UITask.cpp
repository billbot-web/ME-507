#include <Arduino.h>
#include "UITask.h"
#include "EncoderTask.h"  // for EncoderTask::Command enum

// Static instance
UITask* UITask::instance_ = nullptr;

  UITask::UITask(Share<float>* positionShare,
               Share<float>*   velocityShare,
               Share<int8_t>* vref,
               Share<int8_t>*   cmdShare,
               uint32_t        updateMs) noexcept
  : positionShare_(positionShare),
    velocityShare_(velocityShare),
    vref_(vref),
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
  const int8_t code = static_cast<int8_t>(UITask::Command::ZERO);
  instance_->cmdShare_->put(code);
}

void UITask::sendEncoderCmdStart()
{
  if (!instance_ || !instance_->cmdShare_) return;
  const int8_t code = static_cast<int8_t>(UITask::Command::RUN);
  instance_->cmdShare_->put(code);
}

void UITask::sendEncoderCmdStop()
{
  if (!instance_ || !instance_->cmdShare_) return;
  const int8_t code = static_cast<int8_t>(UITask::Command::STOP);
  instance_->cmdShare_->put(code);
}

// ---------------- States ----------------

// WAIT_FOR_INPUT: show prompt; parse serial; forward commands
uint8_t UITask::exec_waitForInput()
{
  if (!instance_) return -1;

  // Print the help menu once when entering WAIT state (avoid spamming every tick)
  if (!instance_->menuPrinted_) {
    Serial.println();
    Serial.print("Enter velocity (numeric), or 'z' to zero encoder: ");
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
      // Check for single-letter zero command
      if ((instance_->inputPos_ == 1) && (instance_->inputBuf_[0] == 'z' || instance_->inputBuf_[0] == 'Z')) {
        sendEncoderCmdZero();
        // stay in WAIT, allow new input
        instance_->inputPos_ = 0;
        instance_->inputBuf_[0] = '\0';
        Serial.println();
        Serial.print("Enter velocity (numeric), or 'z' to zero encoder: ");
        continue;
      }

      // Try parse numeric (signed) value for vref
      char* endptr = nullptr;
      long val = strtol(instance_->inputBuf_, &endptr, 10);
      if (endptr != instance_->inputBuf_ && (*endptr == '\0')) {
        // clamp to -100..100 and publish to vref share
        if (val > 100) val = 100;
        if (val < -100) val = -100;
        // publish to vref share if available
        if (instance_->vref_) {
          instance_->vref_->put(static_cast<int8_t>(val));
          Serial.print("[UITask] published vref = "); Serial.println(val);
            // tell encoder/motor to start running via command share
            instance_->cmdShare_->put(static_cast<int8_t>(UITask::Command::RUN));
        } else {
          Serial.println("[UITask] vref share not available");
        }
       
        // transition to velocity run
        instance_->menuPrinted_ = false; // reset for next WAIT
        return static_cast<int>(VELOCITY_RUN);
      } else {
        Serial.println();
        Serial.println("Invalid number, try again");
        instance_->inputPos_ = 0;
        instance_->inputBuf_[0] = '\0';
        Serial.print("Enter velocity (numeric), or 'z' to zero encoder: ");
        continue;
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
        sendEncoderCmdStop();
        return static_cast<int>(WAIT_FOR_INPUT);
      case 'z': case 'Z':
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


