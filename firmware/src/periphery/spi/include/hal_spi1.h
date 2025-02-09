#ifndef TEST_F411_PERIPHERY_SPI_HAL_SPI_H_
#define TEST_F411_PERIPHERY_SPI_HAL_SPI_H_

#include <cstdint>

#include "stm32g4xx_ll_spi.h"
#include "stm32g4xx_ll_gpio.h"
#include "stm32g4xx_ll_bus.h"

#include "FreeRTOS.h"
#include "semphr.h"

enum spi_polarity_ {
  SPI_POLARITY_LOW,
  SPI_POLARITY_HIGH
};

class HAL_SPI1 {
 private:
  bool is_initialized = false;
  bool is_communication_started = false;
  HAL_SPI1() {};
  ~HAL_SPI1() {};
  HAL_SPI1(const HAL_SPI1 &obj);
  HAL_SPI1 &operator=(const HAL_SPI1 &obj);
  SemaphoreHandle_t semaphore_handle = nullptr;
  enum spi_polarity_ clock_polarity = SPI_POLARITY_LOW;
  uint32_t time_out = 20; // 20 ms to transmit/receive
  void init();
  void configure();

 public:
  static HAL_SPI1 &getInstance() {
      static HAL_SPI1 _instance;
      _instance.init();
      return _instance;
  }
  int32_t exchange_bytes_duplex(uint8_t *, size_t, uint8_t *, size_t);
  int32_t exchange_bytes_half_duplex(uint8_t *, size_t, uint8_t *, size_t);
  int32_t start_communication(enum spi_polarity_);
  int32_t stop_communication();
};
#endif //TEST_F411_PERIPHERY_SPI_HAL_SPI_H_
