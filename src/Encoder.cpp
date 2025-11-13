#include "Encoder.h"
#include "driver/gpio.h"

/**
 * @brief Construct encoder with pins/unit; hardware setup happens in begin().
 */
Encoder::Encoder(gpio_num_t pinA, gpio_num_t pinB, pcnt_unit_t unit)
{
  a_ = pinA; 
  b_ = pinB; 
  unit_ = unit;
  // Initialize time and previous counter snapshot
  prev_time_  = micros();
  // Read an initial counter value after begin(); for now leave 0.
  prev_count_ = 0;
  GR_ = 70*32; // Gear ratio default 70:1 with 32 CPR encoder 2240 counts/rev
}

/**
 * @brief Low-level helper to configure one PCNT channel.
 */
bool Encoder::configChannel(pcnt_channel_t ch,
                            gpio_num_t pulse_gpio,
                            gpio_num_t ctrl_gpio,
                            pcnt_count_mode_t pos_mode,
                            pcnt_count_mode_t neg_mode,
                            pcnt_ctrl_mode_t lctrl_mode,
                            pcnt_ctrl_mode_t hctrl_mode)
{
  pcnt_config_t cfg = {};
  cfg.pulse_gpio_num = pulse_gpio;
  cfg.ctrl_gpio_num  = ctrl_gpio;
  cfg.unit           = unit_;
  cfg.channel        = ch;
  cfg.counter_h_lim  =  32767;  // signed 16-bit max
  cfg.counter_l_lim  = -32768;  // signed 16-bit min

  if (pcnt_unit_config(&cfg) != ESP_OK) return false;
  if (pcnt_set_mode(unit_, ch, pos_mode, neg_mode, lctrl_mode, hctrl_mode) != ESP_OK) return false;
  return true;
}

/**
 * @brief Configure PCNT for true X4 quadrature.
 */
bool Encoder::begin()
{
  // Configure GPIO pins with internal pull-ups
  gpio_config_t io_conf = {};
  io_conf.intr_type = GPIO_INTR_DISABLE;
  io_conf.mode = GPIO_MODE_INPUT;
  io_conf.pin_bit_mask = (1ULL << a_) | (1ULL << b_);
  io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
  
  if (gpio_config(&io_conf) != ESP_OK) {
    return false;
  }
  
  // CH0: pulse = A, ctrl = B → INC on pos, DEC on neg, flip when B is high
  if (!configChannel(PCNT_CHANNEL_0, a_, b_,
                     PCNT_COUNT_INC, PCNT_COUNT_DEC,
                     PCNT_MODE_KEEP, PCNT_MODE_REVERSE)) {
    return false;
  }

  // CH1: pulse = B, ctrl = A → mirror; flip when A is low (complements CH0)
  if (!configChannel(PCNT_CHANNEL_1, b_, a_,
                     PCNT_COUNT_INC, PCNT_COUNT_DEC,
                     PCNT_MODE_REVERSE, PCNT_MODE_KEEP)) {
    return false;
  }


  // Start fresh
  pcnt_counter_pause(unit_);
  pcnt_counter_clear(unit_);
  pcnt_counter_resume(unit_);

  // Initialize state windows/time and prev_count from hardware
  position_ = 0;
  delta_ = 0;
  pos_list_.fill(0);
  t_list_.fill(0);
  prev_time_  = micros();

  int16_t cur = 0;
  pcnt_get_counter_value(unit_, &cur);
  prev_count_ = static_cast<int32_t>(cur);

  return true;
}

/**
 * @brief Perform one update: time delta, read counter, compute wrapped delta, smooth, accumulate.
 */
void Encoder::update()
{
  // --- time delta (like ticks_diff) ---
  const uint32_t now = micros();
  dt_us_ = now - prev_time_;
  prev_time_ = now;

  // --- read current signed 16-bit count ---
  int16_t raw = 0;
  pcnt_get_counter_value(unit_, &raw);
  const int32_t cur_count = static_cast<int32_t>(raw);

  // --- compute delta with wrap handling across [-32768..32767] ---
  // Standard wrap-around detection for signed 16-bit counter
  int32_t delta = cur_count - prev_count_;
  
  
  prev_count_ = cur_count;
  constexpr int32_t ARrange = 32768; // Full range for 16-bit
  // Wrap detection: use proper thresholds for 16-bit signed wrap
  if (delta > ARrange/2) {
    // Forward wrap: counter went from high positive to low negative
    delta -= ARrange;
  } else if (delta < -ARrange/2) {
    // Backward wrap: counter went from low negative to high positive  
    delta += ARrange;
  }

  delta_ = delta;

  // --- sliding windows for smoothing ---
  slide_and_push(pos_list_, delta_);
  slide_and_push(t_list_,  dt_us_);

  // --- accumulate position with overflow protection ---
  // Check for potential overflow before adding
  if ((delta_ > 0 && position_ > INT32_MAX - delta_) ||
      (delta_ < 0 && position_ < INT32_MIN - delta_)) {
    // Position would overflow, reset to prevent undefined behavior
    position_ = delta_;  // Start fresh from current delta
  } else {
    position_ += delta_;
  }
}

/**
 * @brief Get total accumulated position (counts).
 */
float Encoder::get_position() const
{
  return position_/GR_;
}

/**
 * @brief Smoothed velocity (counts per microsecond) using last 5 samples.
 */
double Encoder::get_velocity() const
{
  int64_t pos_sum = 0;
  uint64_t t_sum  = 0;
  for (size_t i = 0; i < WIN; ++i) {
    pos_sum += static_cast<int64_t>(pos_list_[i]);
    t_sum   += static_cast<uint64_t>(t_list_[i]);
  }
  if (t_sum == 0) return 0.0;
  return (static_cast<double>(pos_sum) / (static_cast<double>(t_sum)*GR_));
}

/**
 * @brief Zero the accumulated position (does not clear hardware counter).
 */
void Encoder::zero()
{
  position_ = 0;
}

/**
 * @brief Slide window left and append a new value (drop oldest).
 */
template <typename T>
void Encoder::slide_and_push(std::array<T, WIN>& arr, T value)
{
  for (size_t i = 1; i < WIN; ++i) {
    arr[i - 1] = arr[i];
  }
  arr[WIN - 1] = value;
}
