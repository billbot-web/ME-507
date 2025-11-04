#include <Arduino.h>

//Motor driver class (IN1, IN2)

class Motor {
public:
  Motor(int in1, int in2, int ledcCh1, int ledcCh2,
        uint32_t pwmFreq = 20000, uint8_t resolutionBits = 10)
    : in1_(in1), in2_(in2),
      ch1_(ledcCh1), ch2_(ledcCh2),
      resBits_(resolutionBits),
      maxDuty_((1 << resolutionBits) - 1)
  {
    pinMode(in1_, OUTPUT);
    pinMode(in2_, OUTPUT);
    digitalWrite(in1_, LOW);
    digitalWrite(in2_, LOW);

    ledcSetup(ch1_, pwmFreq, resBits_);
    ledcSetup(ch2_, pwmFreq, resBits_);
    ledcAttachPin(in1_, ch1_);
    ledcAttachPin(in2_, ch2_);
  }

  // effort in [-100,100]
  void setEff(int effort) {
    if (effort > 100) effort = 100;
    if (effort < -100) effort = -100;

    uint32_t duty = (abs(effort) * maxDuty_) / 100;

    if (effort > 0) {
      ledcWrite(ch1_, duty);   // PWM on IN1
      ledcWrite(ch2_, 0);      // LOW on IN2
    } else if (effort < 0) {
      ledcWrite(ch1_, 0);      // LOW on IN1
      ledcWrite(ch2_, duty);   // PWM on IN2
    } else {
      // brake to GND
      ledcWrite(ch1_, 0);
      ledcWrite(ch2_, 0);
    }
  }

  void stop() { setEff(0); }

private:
  int in1_, in2_;
  int ch1_, ch2_;
  uint8_t resBits_;
  uint32_t maxDuty_;
};
