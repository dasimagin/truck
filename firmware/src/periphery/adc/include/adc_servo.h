#ifndef TRUCK_HW_PERIPHERY_ADC_INCLUDE_ADC_H_
#define TRUCK_HW_PERIPHERY_ADC_INCLUDE_ADC_H_

#include <cstdint>

#include "stm32g4xx_ll_adc.h"
#include "stm32g4xx_ll_dma.h"

enum class ADCServoType : uint8_t {
  ADC_SERVO_1 = 1,
  ADC_SERVO_2
};

class ADC {
 private:
  ADC() {};
  ~ADC() {};
  ADC(const ADC &obj);
  ADC &operator=(const ADC &obj);

  ADC_TypeDef *adc_handle = ADC2;
  DMA_TypeDef *dma_handle = DMA1;
  const uint32_t dma_channel = LL_DMA_CHANNEL_5;
  const uint32_t dma_periph_request = LL_DMAMUX_REQ_ADC2;
  const uint32_t resolution = 12;
  uint16_t data[2];
  bool is_initialized = false;
  bool is_started = false;

 public:
  uint32_t init();
  uint32_t start_measurment();
  uint32_t stop_measurment();
  static ADC &getInstance();
  uint32_t get_value(ADCServoType type, uint16_t &value);
};

#endif /* TRUCK_HW_PERIPHERY_ADC_INCLUDE_ADC_H_ */