#include "tle5012.h"
#include <cstdio>

int32_t TLE5012::read_data_from_sensor(uint16_t cmd, uint8_t response[]) {
    int32_t status = 0;
    uint8_t send_buffer[2];
    cmd_to_buffer(cmd, send_buffer);
    if (spi.start_communication(spi_polarity) != 0) {
        status = 1;
    }
    enable_cs();
    status = spi.exchange_bytes_half_duplex(send_buffer, 2, response, 4);
    disable_cs();
    if (spi.stop_communication() != 0) {
        status = 2;
    }
    return status;
}

inline void TLE5012::enable_cs() {
    hal_gpio_reset_pin(cs_port, cs_pin);
}

inline void TLE5012::disable_cs() {
    hal_gpio_set_pin(cs_port, cs_pin);
}

inline void TLE5012::cmd_to_buffer(uint16_t cmd, uint8_t buff[2]) {
    buff[0] = (cmd & 0xFF00) >> 8;
    buff[1] = cmd & 0xFF;
}

TLE5012::TLE5012() : spi(HAL_SPI1::getInstance()) {
    hal_gpio_init(cs_port, cs_pin);
    disable_cs();
};

int32_t TLE5012::read_angle_value(int32_t &angle) {
    int32_t status = 0;
    uint8_t response[2];
    status = read_data_from_sensor(READ_ANGLE_VAL_CMD, response);
    if (status == 0) {
        if ((response[0] & 0x80) == 0) {
            status = 1;  // value wasn't update after last read
            printf("OLD\n");
        }
        angle = ((response[0] << 8) | response[1]) & 0x7FFF; // reverse bytes and delete 16th bit
    }
    return status;
}

int32_t TLE5012::read_angle_speed(int32_t &speed) {
    int32_t status = 0;
    uint8_t response[2];
    status = read_data_from_sensor(READ_ANGLE_SPD_CMD, response);
    if (status == 0) {
        if ((response[0] & 0x80) == 0) {
            status = 1;  // value wasn't update after last read
        }
        speed = ((response[0] << 8) | response[1]) & 0x7FFF;
        if ((response[0] & 0x40)) { // negative speed
            speed -= 0x8000;
        }
    }
    return status;
}

int32_t TLE5012::read_angle_revolution(int32_t &revolution) {
    int32_t status = 0;
    uint8_t response[2];
    status = read_data_from_sensor(READ_ANGLE_SPD_CMD, response);
    if (status == 0) {
        if ((response[0] & 0x80) == 0) {
            status = 1;  // value wasn't update after last read
        }
        revolution = response[1];
        if ((response[0] & 0x01)) { // ccw revolutions
            revolution -= 0x1FF;
        }
    }
    return status;
}
