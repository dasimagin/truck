#include "SensorPolling.h"

#include <cstdio>
#include <cstring>

#include "FreeRTOS.h"
#include "task.h"
#include "COBS.h"

#include "board.h"
#include "usart2.h"

#include "wheel_encoder.h"
#include "ServoController.h"

#define ENCODERS_SPEED_MSG_TAG           1
#define SERVO_MEASURED_ANGLES_MSG_TAG    2

#define SERVO_TARGET_ANGLES_MSG_TAG      3

static uint8_t cobs_buffer[64] = {0};
static uint8_t *msg_buff = &(cobs_buffer[1]);

static uint8_t rx_cmd_buffer[64] = { 0 };
volatile bool rx_data_available = false;

static void send_cobs_buffer(uint32_t size) {
    cobs::encode(cobs_buffer, size + 1);
    cobs_buffer[size + 1] = 0x00;
    usart_2_send_buffer(cobs_buffer, size + 2);
}

SensorPolling& SensorPolling::getInstance() {
    static SensorPolling _instance;
    return _instance;
}

void SensorPolling::task() {
    static WheelEncoder &enc_left_front = WheelEncoder::get_instance(WheelType::LEFT_FRONT);
    static WheelEncoder &enc_right_front = WheelEncoder::get_instance(WheelType::RIGHT_FRONT);
    enc_left_front.init();
    enc_right_front.init();

    static ServoController &SC = ServoController::getInstance();
    volatile uint32_t last_msg_send = board_get_tick();

    usart_2_set_memory_for_rx_data(rx_cmd_buffer, 64);

    while (true) {
        if (board_get_tick() != last_msg_send) {
            uint16_t left_angle = 0;
            uint16_t right_angle = 0;
            SC.get_angle(left_angle, right_angle);
            msg_buff[0] = SERVO_MEASURED_ANGLES_MSG_TAG;
            memcpy(&(msg_buff[1]), &left_angle, sizeof(left_angle));
            memcpy(&(msg_buff[3]), &right_angle, sizeof(right_angle));
            send_cobs_buffer(5);

            float left_speed = enc_left_front.get_ticks_per_sec();
            float right_speed = enc_right_front.get_ticks_per_sec();
            msg_buff[0] = ENCODERS_SPEED_MSG_TAG;
            memcpy(&(msg_buff[1]), &left_speed, sizeof(left_speed));
            memcpy(&(msg_buff[5]), &right_speed, sizeof(right_speed));
            send_cobs_buffer(9);

            last_msg_send = board_get_tick();
        }

        if (rx_data_available == true) {
            uint8_t *rx_data = NULL;
            uint32_t rx_bytes = 0;
            usart_2_get_memory_for_rx_data(&rx_data, &rx_bytes);
            if ((rx_bytes == 0) || (rx_bytes == 64)) {
                // printf("Empty message\n");
            } else {
                uint8_t *rx_data_end = rx_data + rx_bytes;
                uint8_t *curent_msg = rx_data;
                do {
                    uint32_t msg_size = 0;

                    for (msg_size = 0; ((curent_msg + msg_size) < rx_data_end)
                        && (curent_msg[msg_size] != 0); ++msg_size);

                    if ((curent_msg + msg_size) < rx_data_end) {
                        cobs::decode(curent_msg,  msg_size);
                    }
                    uint8_t *cmd_data = curent_msg + 1;
                    uint8_t cmd_size = msg_size - 1;
                    if ((cmd_data[0] == 3) && (cmd_size == 3)) {
                        uint8_t left_servo_angle = 0;
                        uint8_t right_servo_angle = 0;
                        left_servo_angle = cmd_data[1];
                        right_servo_angle = cmd_data[2];
                        SC.set_angle(left_servo_angle, right_servo_angle);
                        // printf("ang %d %d\n", left_servo_angle, right_servo_angle);
                    } else {
                        // printf("Incorrect request\n");
                    }
                    curent_msg += (msg_size + 1);
                } while (curent_msg < rx_data_end);
            }
            rx_data_available = false;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

int32_t SensorPolling::init() {
    int32_t status = 0;
    if (!is_initialized) {
        BaseType_t task_status = xTaskCreate(startTaskImpl, "sensor_task", 1024, this, 1, &task_handle);
        if (task_status == pdTRUE) {
            vTaskSuspend(task_handle);
//            imu.init();
            is_initialized = true;
        } else {
            printf("Error сreating sensor polling task\n");
            status = 2;
        }
    } else {
        status = 1;
    }
    return status;
}

void SensorPolling::start() {
    vTaskResume(task_handle);
}

int32_t SensorPolling::get_gyro_data(float &x_axis, float &y_axis, float &z_axis) {
    int32_t status = 0;
    if (is_initialized) {
        x_axis = gyro_x;
        y_axis = gyro_y;
        z_axis = gyro_z;
        if (!gyro_updated) {
            status = 2;
        }
    } else {
        status = 1;
    }
    return status;
}

int32_t SensorPolling::get_accel_data(float &x_axis, float &y_axis, float &z_axis) {
    int32_t status = 0;
    if (is_initialized) {
        x_axis = accel_x;
        y_axis = accel_y;
        z_axis = accel_z;
        if (!accel_updated) {
            status = 2;
        }
    } else {
        status = 1;
    }
    return status;
}

int32_t SensorPolling::get_magn_data(float &x_axis, float &y_axis, float &z_axis) {
    int32_t status = 0;
    if (is_initialized) {
        x_axis = magn_x;
        y_axis = magn_y;
        z_axis = magn_z;
        if (!magn_updated) {
            status = 2;
        }
    } else {
        status = 1;
    }
    return status;
}
