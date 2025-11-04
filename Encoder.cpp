#include <Arduino.h>
#include "driver/pcnt.h"

// ---------------- Encoder via PCNT (x2 on channel A) ----------------
class Encoder {
public:
  // edgeMultiplier = 2 → count A rising+falling (x2). Use 1 for rising-only.
  Encoder(gpio_num_t pinA, gpio_num_t pinB,
              int16_t lowLimit = -32767, int16_t highLimit = 32768,
              int edgeMultiplier = 2)
    : a_(pinA), b_(pinB), low_(lowLimit), high_(highLimit), mult_(edgeMultiplier) {}

  bool begin(pcnt_unit_t unit = PCNT_UNIT_0) {
    unit_ = unit;

    pcnt_config_t cfg = {};
    cfg.pulse_gpio_num = a_;
    cfg.ctrl_gpio_num  = b_;
    cfg.unit           = unit_;
    cfg.channel        = PCNT_CHANNEL_0;
    cfg.counter_h_lim  = high_;
    cfg.counter_l_lim  = low_;
    // Base config; exact modes set below
    if (pcnt_unit_config(&cfg) != ESP_OK) return false;

    // Mirror channel for falling edges on A (x2)
    cfg.channel = PCNT_CHANNEL_1;
    if (pcnt_unit_config(&cfg) != ESP_OK) return false;

    // Glitch filter (~12.5 us @ 80MHz)
    pcnt_set_filter_value(unit_, 1000);
    pcnt_filter_enable(unit_);

    // Configure modes:
    // Count on A edges; direction from level of B.
    // When B is LOW → INC on A edges; when B is HIGH → reverse (i.e., DEC).
    pcnt_set_mode(unit_, PCNT_CHANNEL_0,
                  PCNT_COUNT_INC,                      // pos edge on A
                  (mult_ >= 2) ? PCNT_COUNT_DEC : PCNT_COUNT_DIS, // neg edge on A (x2)
                  PCNT_MODE_REVERSE,                   // B low = normal
                  PCNT_MODE_KEEP);                     // B high = reverse

    pcnt_set_mode(unit_, PCNT_CHANNEL_1,
                  PCNT_COUNT_INC,
                  (mult_ >= 2) ? PCNT_COUNT_DEC : PCNT_COUNT_DIS,
                  PCNT_MODE_REVERSE,
                  PCNT_MODE_KEEP);

    pcnt_counter_pause(unit_);
    pcnt_counter_clear(unit_);
    pcnt_counter_resume(unit_);
    total_ = 0;
    return true;
  }

  // Read delta since last clear, then clear
  int32_t readAndReset() {
    int16_t v = 0;
    pcnt_get_counter_value(unit_, &v);
    pcnt_counter_clear(unit_);
    total_ += v;
    return (int32_t)v;
  }

  int32_t total() const { return total_; }

private:
  gpio_num_t a_, b_;
  int16_t low_, high_;
  int mult_;
  pcnt_unit_t unit_;
  int32_t total_ = 0;
};
