#ifndef TRUCK_HW_PERIPHERY_ENCODER_TIMER_INCLUDE_ENCODER_TIMER_H_
#define TRUCK_HW_PERIPHERY_ENCODER_TIMER_INCLUDE_ENCODER_TIMER_H_

#include <cstdint>
#include <unordered_map>
#include "stm32g4xx_ll_tim.h"
#include "stm32g4xx_ll_dma.h"
#include <array>
#include <vector>

enum class EncoderType : uint8_t {
  ID_0,
  ID_1,
  ID_2,
  ID_3
};

class EncoderTimer {
 private:
  enum EncoderType type;
  static bool is_common_tim_initialized;
  const TIM_TypeDef *common_timer_handle = TIM2;
  const DMA_TypeDef *common_dma_handler = DMA1;
  static constexpr uint32_t APB1_CLOCK = 144000000;
  static constexpr uint32_t TIM_PRESCALER = 143;
  static constexpr uint32_t TIM_AUTORELOAD = 4294967296-1;
  static constexpr uint32_t max_data_size = 50;
  uint32_t last_cnt_value = 0;
  volatile bool &buffer_overwrite;
  std::array<int, max_data_size> raw_buffer;
  std::array<int, max_data_size>::iterator prev_get_it = raw_buffer.begin();
  uint32_t timer_channel;
  uint32_t dma_channel;

//  GPIO_TypeDef *input_port;
//  uint32_t *input_pin;

  EncoderTimer(EncoderType id);
  ~EncoderTimer() {};
  EncoderTimer(const EncoderTimer &obj) = delete;
  EncoderTimer &operator=(const EncoderTimer &obj) = delete;

 public:
  static EncoderTimer &get_instance(EncoderType id) {
      static std::unordered_map<EncoderType, EncoderTimer *> instances;
      auto it = instances.find(id);
      if (it == instances.end()) {
          instances[id] = new EncoderTimer(id);
      }
      return *instances[id];
  }

  int init();
  uint32_t get_data_size();
  uint32_t get_data(std::vector<int> &output_data);
  constexpr float get_tick_len_micros() {
    return (1000000.0f) / (APB1_CLOCK / (TIM_PRESCALER+1));
  }
};

#endif //TRUCK_HW_PERIPHERY_ENCODER_TIMER_INCLUDE_ENCODER_TIMER_H_
