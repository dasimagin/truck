#ifndef TRUCK_HW_CHIPSET_ADS1115_INCLUDE_ADS1115_H_
#define TRUCK_HW_CHIPSET_ADS1115_INCLUDE_ADS1115_H_

#include <cstdint>

#include "i2c2.h"

enum class ADCServoType : uint8_t {
  ADC_SERVO_1 = 1,
  ADC_SERVO_2
};

class ADS1115 {
 private:
  ADS1115(ADCServoType id);
  ~ADS1115() {};
  ADS1115(const ADS1115 &obj);
  ADS1115 &operator=(const ADS1115 &obj);

  I2C &i2c_handle = I2C::getInstance();
  uint8_t i2c_address = 0x00;
  const uint8_t i2c_timeout = 10;
  bool is_initialized = false;

  uint32_t config(void);

 public:
  uint32_t init();
  static ADS1115 &get_instance(ADCServoType type);
  uint32_t read_value(uint16_t& value);
};

#endif /* TRUCK_HW_CHIPSET_ADS1115_INCLUDE_ADS1115_H_ */