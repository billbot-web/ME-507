#pragma once
#include <Arduino.h>
#include "driver/pcnt.h"
#include <array>

/**
 * @class Encoder
 * @brief ESP32 PCNT-based quadrature encoder reader (X4), matching a MicroPython-style API.
 *
 * Behavior mirrors the provided Python class:
 *  - update(): reads current hardware counter and time, computes delta with wrap handling,
 *              updates a 5-sample smoothing window, accumulates position
 *  - get_position(): returns accumulated position (counts)
 *  - get_velocity(): returns smoothed velocity (counts per microsecond) like the Python version
 *  - zero(): sets accumulated position to zero
 *
 * Hardware:
 *  - Uses one PCNT unit with both channels for true X4:
 *      CH0: pulse = A, ctrl = B  (flip when B is high)
 *      CH1: pulse = B, ctrl = A  (flip when A is low)
 *  - Counter range is signed 16-bit [-32768, 32767]
 *
 * Notes:
 *  - Call begin() once after constructing.
 *  - Call update() at a regular cadence (e.g., every 1–10 ms) to refresh velocity smoothing.
 *  - get_velocity() returns counts / microsecond. Multiply by 1e6 for counts / second.
 */
class Encoder {
public:
  /**
   * @brief Construct an Encoder (does not touch hardware yet).
   * @param pinA          GPIO for phase A (must be PCNT-capable)
   * @param pinB          GPIO for phase B (must be PCNT-capable)
   * @param unit          PCNT unit to use (PCNT_UNIT_0 .. PCNT_UNIT_7)
   */
  Encoder(gpio_num_t pinA,
          gpio_num_t pinB,
          pcnt_unit_t unit);

  /**
   * @brief Configure the PCNT hardware for X4 quadrature.
   * @return true on success, false otherwise
   */
  bool begin();

  /**
   * @brief Run one update step: compute dt, read counter, delta (with wrap), smooth, accumulate.
   *
   * Matches the Python semantics:
   *   - Uses micros() for timing (ticks_us())
   *   - Keeps 5-sample windows for delta and dt
   */
  void update();

  /**
   * @brief Total accumulated position (counts), updated by update().
   * @return accumulated counts since startup or zero()
   */
  float get_position() const;

  /**
   * @brief Smoothed velocity (counts per microsecond), like the Python class.
   * @return (sum of recent deltas) / (sum of recent dt in microseconds)
   *
   * Multiply by 1e6 to get counts per second.
   */
  double get_velocity() const;

  /**
   * @brief Set accumulated position to zero (does not clear hardware counter).
   */
  void zero();

private:
  // --- Configuration / hardware binding ---
  gpio_num_t  a_;                 ///< Phase A GPIO
  gpio_num_t  b_;                 ///< Phase B GPIO
  pcnt_unit_t unit_;              ///< PCNT unit used

  // --- State variables ---
  float  position_   = 0;       ///< total accumulated position
  int32_t  prev_count_ = 0;       ///< previous hardware counter snapshot (signed 16-bit)
  int32_t  delta_      = 0;       ///< last delta counts
  uint32_t prev_time_  = 0;       ///< previous timestamp (us)
  uint32_t dt_us_      = 0;       ///< last dt (us)
  uint16_t GR_         = 70*32;   ///< Gear ratio (counts per output revolution)

  static constexpr size_t WIN = 5;              ///< smoothing window length
  std::array<int32_t, WIN>  pos_list_{};        ///< recent deltas
  std::array<uint32_t, WIN> t_list_{};          ///< recent dt (us)

  /**
   * @brief Configure one PCNT channel with edge/count/ctrl modes.
   */
  bool configChannel(pcnt_channel_t ch,
                     gpio_num_t pulse_gpio,
                     gpio_num_t ctrl_gpio,
                     pcnt_count_mode_t pos_mode,
                     pcnt_count_mode_t neg_mode,
                     pcnt_ctrl_mode_t lctrl_mode,
                     pcnt_ctrl_mode_t hctrl_mode);

  /**
   * @brief Slide a fixed-size array left and append a new value (drop oldest).
   */
  template <typename T>
  static void slide_and_push(std::array<T, WIN>& arr, T value);
};
