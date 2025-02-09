#include "icm20984.h"

#include <cstdio>

int32_t ICM20984::read_data_from_sensor(uint8_t register_address, uint8_t *ptr_response, uint8_t response_size) {
    uint8_t read_register_address = READ_FLAG | register_address;
    int32_t status = 0;
    enable_cs();
    status = spi.exchange_bytes_duplex(&read_register_address, 1, ptr_response, response_size);
    disable_cs();
    return status;
}

int32_t ICM20984::write_data_to_sensor(uint8_t *request, uint8_t request_size) {
    uint8_t response;
    int32_t status = 0;
    enable_cs();
    status = spi.exchange_bytes_duplex(request, request_size, &response, 0);
    disable_cs();
    return status;
}

void get_x_y_z_values(uint8_t *buffer, float resolution, float &x_value, float &y_value, float &z_value) {
    x_value = (float) ((int16_t) ((buffer[0] << 8) | buffer[1])) * resolution;
    y_value = (float) ((int16_t) ((buffer[2] << 8) | buffer[3])) * resolution;
    z_value = (float) ((int16_t) ((buffer[4] << 8) | buffer[5])) * resolution;
}

int32_t ICM20984::read_mems_values(uint8_t *response_buffer) {
    int32_t status = 0;
    do {
        if (read_data_from_sensor(ACCEL_XOUT_H, response_buffer, 12) != 0) {
            status = 1;
            break;
        }
        for (int i = 0; i < 6; ++i) {
//            printf("%X-", (uint16_t) ((response_buffer[2*i] << 8) | response_buffer[2*i+1]));
        }
//        printf("\n");
    } while (false);
    return status;
}

int32_t ICM20984::update_values() {
    int32_t status = 0;
    do {
        if (spi.start_communication(spi_polarity) != 0) {
            status = 1;
            break;
        }
        uint8_t raw_values[12];
        if (read_mems_values(raw_values) != 0) {
            status = 2;
            break;
        }
        get_x_y_z_values(raw_values, 2.0f / INT16_MAX, accel_x, accel_y, accel_z);
        get_x_y_z_values(&(raw_values[6]), 250.0f / INT16_MAX, gyro_x, gyro_y, gyro_z);
    } while (false);
    status |= spi.stop_communication();

    return status;
}

int32_t ICM20984::get_gyro_values(float &x_axis, float &y_axis, float &z_axis) {
    x_axis = gyro_x;
    y_axis = gyro_y;
    z_axis = gyro_z;
    return 0;
}

int32_t ICM20984::get_accel_values(float &x_axis, float &y_axis, float &z_axis) {
    x_axis = accel_x;
    y_axis = accel_y;
    z_axis = accel_z;
    return 0;
}

int32_t ICM20984::init2() {
    int32_t status = 0;
    do {
        if (spi.start_communication(spi_polarity) != 0) {
            status = 1;
            break;
        }
        uint8_t who_am_i[1];
        uint8_t bank_buffer[1];
        if (write_data_to_sensor(bank_buffer, 2) != 0) {
            status = 1;
        }
        read_data_from_sensor(0, who_am_i, 1);
        if (who_am_i[0] == 0xEA) {
            printf("WHO AM I true\n");
        } else {
            printf("%d\n", who_am_i[0]);
        }
        if (write_data_to_sensor(bank_buffer, 2) != 0) {
            status = 1;
        }
        uint8_t reset_buffer[2] = {6, 0x80 | 0x41}; // Reset
        if (write_data_to_sensor(reset_buffer, 2) != 0) {
            status = 2;
            break;
        }
        for (volatile int i = 0; i < 100000; ++i);
//        if (write_data_to_sensor(bank_buffer, 2) != 0) {
//            status = 1;
//        }
        uint8_t wakeup_buffer[2] = {6, 0x01}; // Wakeup
        if (write_data_to_sensor(wakeup_buffer, 2) != 0) {
            status = 2;
            break;
        }
        for (volatile int i = 0; i < 100000; ++i);
//        if (write_data_to_sensor(bank_buffer, 2) != 0) {
//            status = 1;
//        }
//        for (volatile int i = 0; i < 1000; ++i);
        uint8_t i2c_disable_buffer[2] = {3, 0x10}; // Disable I2C
        if (write_data_to_sensor(i2c_disable_buffer, 2) != 0) {
            status = 2;
            break;
        }
        set_register_bank(2);
        uint8_t odr_enable[2] = {9, 0x01}; // Disable I2C
        if (write_data_to_sensor(odr_enable, 2) != 0) {
            status = 2;
            break;
        }
        set_register_bank(0);
    } while (false);
    status |= spi.stop_communication();
    printf("status %d\n", status);
    return status;
}


int32_t ICM20984::init() {
    int32_t status = 0;
    do {
        if (spi.start_communication(spi_polarity) != 0) {
            status = 1;
            break;
        }
        int32_t retries = 3;
        uint8_t who_am_i[1];
        do {
            uint8_t reset_buffer[2] = {6, 0x80 | 0x41}; // Reset
            if (write_data_to_sensor(reset_buffer, 2) == 0) {
                for (volatile int i = 0; i < 100000; ++i);
                read_data_from_sensor(0, who_am_i, 1);
                if (who_am_i[0] == 0xEA) {
                    printf("WHO AM I true\n");
                    break;
                } else {
                    printf("%d\n", who_am_i[0]);
                }
            }
//            vTaskDelay(pdMS_TO_TICKS(5));
            --retries;
        } while (retries > 0);

        if (who_am_i[0] != 0xEA) {
            status = 2;
            break;
        }
        set_register_bank(0);
        uint8_t wakeup_buffer[2] = {6, 0x01}; // Wakeup
        if (write_data_to_sensor(wakeup_buffer, 2) != 0) {
            status = 2;
            break;
        }
        for (volatile int i = 0; i < 100000; ++i);
        uint8_t i2c_disable_buffer[2] = {3, 0x10}; // Disable I2C
        if (write_data_to_sensor(i2c_disable_buffer, 2) != 0) {
            status = 2;
            break;
        }
        set_register_bank(2);
        uint8_t odr_enable[2] = {9, 0x01}; // ODR Enable
        if (write_data_to_sensor(odr_enable, 2) != 0) {
            status = 2;
            break;
        }

//        accel_gyro_calibration();
        set_register_bank(0);
    } while (false);
    status |= spi.stop_communication();

    return status;
}

int32_t ICM20984::accel_gyro_calibration() {
    int32_t gyro_x_bias = 0;
    int32_t gyro_y_bias = 0;
    int32_t gyro_z_bias = 0;
    int32_t accel_x_bias = 0;
    int32_t accel_y_bias = 0;
    int32_t accel_z_bias = 0;

    uint8_t raw_values[12];
    for (int i = 0; i < 128; ++i) {
        read_mems_values(raw_values);
        gyro_x_bias += (int16_t)(raw_values[6] << 8 | raw_values[7]);
        gyro_y_bias += (int16_t)(raw_values[8] << 8 | raw_values[9]);
        gyro_z_bias += (int16_t)(raw_values[10] << 8 | raw_values[11]);
        accel_x_bias += (int16_t)(raw_values[0] << 8 | raw_values[1]);
        accel_y_bias += (int16_t)(raw_values[2] << 8 | raw_values[3]);
        accel_z_bias += (int16_t)(raw_values[4] << 8 | raw_values[5]);
        for (volatile int ij = 0; ij < 10000; ++ij);
    }
    gyro_x_bias = gyro_x_bias / 128 / 4;
    gyro_y_bias = gyro_y_bias / 128 / 4;
    gyro_z_bias = gyro_z_bias / 128 / 4;

    uint8_t gyro_offset[6] = {0};

    gyro_offset[0] = (-gyro_x_bias >> 8) & 0xFF;
    gyro_offset[1] = (-gyro_x_bias)       & 0xFF;
    gyro_offset[2] = (-gyro_y_bias >> 8) & 0xFF;
    gyro_offset[3] = (-gyro_y_bias)       & 0xFF;
    gyro_offset[4] = (-gyro_z_bias >> 8) & 0xFF;
    gyro_offset[5] = (-gyro_z_bias)       & 0xFF;

    accel_x_bias = - (accel_x_bias / 128 / 8);
    accel_y_bias = - (accel_y_bias / 128 / 8);
    accel_z_bias = - (accel_z_bias / 128 / 8);

    uint8_t accel_offset[6] = {0};

    accel_offset[0] = (accel_x_bias >> 8) & 0xFF;
    accel_offset[1] = (accel_x_bias)      & 0xFE;
    accel_offset[2] = (accel_y_bias >> 8) & 0xFF;
    accel_offset[3] = (accel_y_bias)      & 0xFE;
    accel_offset[4] = (accel_z_bias >> 8) & 0xFF;
    accel_offset[5] = (accel_z_bias)      & 0xFE;

    uint8_t register_id;
    set_register_bank(2);
    register_id = 3;
    write_data_to_sensor(&register_id, 1);
    write_data_to_sensor(gyro_offset, 6);

    set_register_bank(1);
    register_id = 0x14;
    write_data_to_sensor(&register_id, 1);
    write_data_to_sensor(gyro_offset, 6);

    return 0;
}

int32_t ICM20984::set_register_bank(uint8_t bank) {
    int32_t status = 0;
    uint8_t bank_buffer[2] = {0x7f, bank}; // bank0
    if (write_data_to_sensor(bank_buffer, 2) != 0) {
        status = 1;
    }
    return status;
}
