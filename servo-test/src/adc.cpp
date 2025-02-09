#include <Arduino.h>
#include "ADS1115.h"
#include "driver/i2c.h"

#define I2C_ADC 1
#define I2C_ADDRESS 0x49

static void i2c_init(void) {
    i2c_config_t confI2C;
    memset(&confI2C, 0U, sizeof(i2c_config_t));

    confI2C.mode = I2C_MODE_MASTER;
    confI2C.sda_io_num = 17;
    confI2C.scl_io_num = 16;
    confI2C.master.clk_speed = 400000;
    confI2C.clk_flags = 0;

    i2c_param_config(I2C_ADC, &confI2C);
    i2c_driver_install(I2C_ADC, I2C_MODE_MASTER, 0, 0, 0);
}

void adc_init() {
    i2c_init();
    uint16_t register_data = 0;
	uint8_t data[3] = {0};

	register_data = ADS1115_COMP_QUE;						// 11  : Disable comparator and set ALERT/RDY pin to high-impedance (default)
	register_data |= (ADS1115_DR_2 | ADS1115_DR_1 | ADS1115_DR_0);	// 111 : 860 SPS
	register_data &= ~ADS1115_MODE;							// 0   : continiuos
	register_data &= ~ADS1115_PGA;							// 000 : FSR = ±6.144 V (187.5μV)
    register_data |= ADS1115_MUX_2;						    // 100 : AINP = AIN0 and AINN = GND
	register_data &= ~ADS1115_OS;							// 0   : no effect

    data[0] = ADS1115_REG_CONFIG;
	data[1] = register_data >> 8;							// msb
	data[2] = register_data & 0xFF;							// lsb

    if (i2c_master_write_to_device(I2C_ADC, I2C_ADDRESS, data, 3, 50) != 0) {
        // Serial.println("error write reg");
    }
}

uint16_t adc_read(uint8_t channel) {
	uint16_t received_data = 0;
	uint8_t data[2] = {0};

    uint8_t reg_data = ADS1115_REG_CONVERSION;
    if (i2c_master_write_read_device(I2C_ADC, I2C_ADDRESS, &reg_data, 1, data, 2, 50) != 0) {
        // Serial.println("error read reg");
    }

	received_data = (uint16_t)((data[0] << 8) | data[1]);

    if (received_data == 0) {
        adc_init();
    }
	return received_data;
}
