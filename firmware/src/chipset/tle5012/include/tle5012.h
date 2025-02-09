// Commands for read
#define READ_STA_CMD_NOSAFETY       0x8000
#define READ_STA_CMD                0x8001
#define READ_ACTIV_STA_CMD          0x8011
#define READ_ANGLE_VAL_CMD          0x8021
#define READ_ANGLE_SPD_CMD          0x8031
#define READ_ANGLE_REV_CMD          0x8041
#define READ_TEMP_CMD               0x8051
#define READ_INTMODE_1              0x8061
#define READ_SIL                    0x8071
#define READ_INTMODE_2              0x8081
#define READ_INTMODE_3              0x8091
#define READ_OFFSET_X               0x80A1
#define READ_OFFSET_Y               0x80B1
#define READ_SYNCH                  0x80C1
#define READ_IFAB                   0x80D1
#define READ_INTMODE_4              0x80E1
#define READ_TEMP_COEFF             0x80F1
#define READ_RAW_X_CMD              0x8101
#define READ_RAW_Y_CMD              0x8111

// Commands for updated read
#define READ_UPD_STA_CMD            0x8401

#include "hal_spi1.h"
#include "hal_gpio.h"
#include "stm32g4xx_ll_gpio.h"

class TLE5012 {
 private:
  HAL_SPI1 &spi;
  const uint32_t cs_port = GPIO_PORT_B;
  const uint32_t cs_pin = GPIO_PIN_2;
  const enum spi_polarity_ spi_polarity = SPI_POLARITY_LOW;

  int32_t read_data_from_sensor(uint16_t cmd, uint8_t response[]);
  inline void enable_cs(); // todo constexpr
  inline void disable_cs();
  inline void cmd_to_buffer(uint16_t cmd, uint8_t buff[2]);
  TLE5012();
  ~TLE5012() {};
  TLE5012(const TLE5012 &obj) = delete;
  TLE5012 &operator=(const TLE5012 &obj) = delete;

 public:

  static TLE5012 &getInstance() {
      static TLE5012 _instance;
      return _instance;
  }
  int32_t read_angle_value(int32_t &angle);
  int32_t read_angle_speed(int32_t &speed);
  int32_t read_angle_revolution(int32_t &revolution);
};
