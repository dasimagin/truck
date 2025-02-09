#ifndef TRUCK_HW_PERIPHERY_I2C_INCLUDE_I2C2_H_
#define TRUCK_HW_PERIPHERY_I2C_INCLUDE_I2C2_H_

#include <cstdint>
#include <cstddef>

#include "FreeRTOS.h"
#include "semphr.h"
#include "stm32g4xx_ll_i2c.h"

uint16_t ADS1115_ReadADC(uint8_t channel);
void ADS1115_config(uint8_t addr);

class I2C {
 private:
  I2C() {};
  ~I2C() {};
  I2C(const I2C &obj);
  I2C &operator=(const I2C &obj);

  I2C_TypeDef *i2c_handle = I2C2;
  bool is_initialized = false;

  SemaphoreHandle_t mutex;

 public:
  uint32_t init();
  static I2C &getInstance();

  uint32_t read_bytes(
    uint8_t chip_address,
    uint8_t register_address,
    uint8_t *ptr_bytes,
    size_t size,
    uint32_t time_out
  );

  uint32_t write_bytes(
    uint8_t chip_address,
    uint8_t register_address,
    uint8_t *ptr_bytes,
    size_t size,
    uint32_t time_out
  );

};

#endif /* TRUCK_HW_PERIPHERY_I2C_INCLUDE_I2C2_H_ */