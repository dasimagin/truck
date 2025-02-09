
#ifndef TRUCK_HW_CHIPSET_WHEEL_ENCODER_INCLUDE_WHEEL_ECNODER_H_
#define TRUCK_HW_CHIPSET_WHEEL_ENCODER_INCLUDE_WHEEL_ECNODER_H_

#include <cstdint>
#include <unordered_map>

#include "encoder_timer.h"

enum class WheelType : uint8_t {
  LEFT_FRONT,
  RIGHT_FRONT,
  LEFT_REAR,
  RIGHT_REAR
};


class WheelEncoder {
 private:
  static constexpr uint32_t idle_timeout_ms = 100;
  static constexpr float low_pass_coef = 0.1f;

  bool is_initialized = false;
  enum WheelType type;
  EncoderTimer& encoder_timer_handle;
  static constexpr uint32_t max_data_size = 50;
  std::vector<int> encoder_ticks;

  float current_speed = 0.0f;
  uint32_t last_tick_ts = 0;

  ~WheelEncoder() {};
  WheelEncoder(const WheelEncoder &obj) = delete;
  WheelEncoder &operator=(const WheelEncoder &obj) = delete;

  static EncoderTimer &get_timer_instance(WheelType wheel_type_) {
      switch (wheel_type_) {
          case WheelType::LEFT_FRONT:
              return EncoderTimer::get_instance(EncoderType::ID_0);
          case WheelType::RIGHT_FRONT:
              return EncoderTimer::get_instance(EncoderType::ID_1);
          case WheelType::LEFT_REAR:
              return EncoderTimer::get_instance(EncoderType::ID_2);
          case WheelType::RIGHT_REAR:
              return EncoderTimer::get_instance(EncoderType::ID_3);
      }
  }

  WheelEncoder(WheelType id) : encoder_timer_handle(get_timer_instance(id)) {};

  void low_pas_filter_apply(float raw_speed);

 public:
  static WheelEncoder &get_instance(WheelType id);

  uint32_t init(void);
  float get_ticks_per_sec(void);
};

#endif //TRUCK_HW_CHIPSET_WHEEL_ENCODER_INCLUDE_WHEEL_ECNODER_H_