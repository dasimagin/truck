//
// Created by Георгий Сенин on 14.03.2024.
//

#ifndef TRUCK_HW_CHIPSET_ICM20984_ICM20984_H_
#define TRUCK_HW_CHIPSET_ICM20984_ICM20984_H_

#define READ_FLAG          0x80
#define ACCEL_XOUT_H       0x2D
#define ACCEL_XOUT_L       0x2E
#define ACCEL_YOUT_H       0x2F
#define ACCEL_YOUT_L       0x30
#define ACCEL_ZOUT_H       0x31
#define ACCEL_ZOUT_L       0x32
#define GYRO_XOUT_H        0x33
#define GYRO_XOUT_L        0x34
#define GYRO_YOUT_H        0x35
#define GYRO_YOUT_L        0x36
#define GYRO_ZOUT_H        0x37
#define GYRO_ZOUT_L        0x38

#include "hal_spi1.h"
#include "hal_gpio.h"

class ICM20984 {
 private:
  HAL_SPI1 &spi;
  const enum spi_polarity_ spi_polarity = SPI_POLARITY_HIGH;
  const uint32_t cs_port = GPIO_PORT_A;
  const uint32_t cs_pin = GPIO_PIN_3;

  float gyro_x, gyro_y, gyro_z;
  float accel_x, accel_y, accel_z;

  inline void enable_cs() {
      hal_gpio_reset_pin(cs_port, cs_pin);
  }
  inline void disable_cs() {
      hal_gpio_set_pin(cs_port, cs_pin);
  }

  ICM20984() : spi(HAL_SPI1::getInstance()) {
      hal_gpio_init(cs_port, cs_pin);
      disable_cs();
  };
  ~ICM20984() {};
  ICM20984(const ICM20984 &obj) = delete;
  ICM20984 &operator=(const ICM20984 &obj) = delete;

  int32_t read_data_from_sensor(uint8_t register_address, uint8_t *response, uint8_t response_size);
  int32_t write_data_to_sensor(uint8_t *request, uint8_t request_size);
  int32_t set_register_bank(uint8_t bank);
  int32_t read_mems_values(uint8_t *raw_values);
  int32_t accel_gyro_calibration();
 public:
  static ICM20984 &getInstance() {
      static ICM20984 _instance;
      return _instance;
  }
  int32_t init();
  int32_t init2();
  int32_t update_values();
  int32_t get_gyro_values(float &x_axis, float &y_axis, float &z_axis);
  int32_t get_accel_values(float &x_axis, float &y_axis, float &z_axis);
};

#endif //TRUCK_HW_CHIPSET_ICM20984_ICM20984_H_
