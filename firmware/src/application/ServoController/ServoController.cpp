#include <cstdio>
#include "ServoController.h"
#include "FreeRTOS.h"
#include "task.h"
#include "board.h"

#define DEFAULT_SERVO_ANGLE     90.0f

ServoController& ServoController::getInstance() {
    static ServoController _instance;
    return _instance;
}

void ServoController::task() {
    while (true){
        // TODO Implement true Ackermann with feedback

        // left_servo.set_angle(left_target_angle);
        // right_servo.set_angle(right_target_angle);
        vTaskDelay(pdMS_TO_TICKS(1000));
        // TODO Set servo health status (does it achieve target angle)?
    }
}

uint32_t ServoController::init() {
    uint32_t status = 0;

    BaseType_t returned = xTaskCreate(startTaskImpl, "servo_task", 512, this, 2, &task_handle);
    if (returned == pdTRUE) {
        if ((left_servo.init() == 0) && (right_servo.init() == 0)) {
            is_initialized = true;
            left_servo.calibrate();
            right_servo.calibrate();
            vTaskResume(task_handle);
        } else {
            status = 2;
        }
    } else {
        status = 1;
    }

    return status;
}

uint32_t ServoController::set_angle(float left_servo_angle, float right_servo_angle) {
    uint32_t status = 0;

    if (is_initialized) {
        left_target_angle = left_servo_angle;
        right_target_angle = right_servo_angle;

        // TODO Remove after implementation true Ackermann with feedback
        uint32_t left_status = left_servo.set_angle(left_target_angle);
        uint32_t right_status = right_servo.set_angle(right_target_angle);
        if ((left_status != 0) || (right_status != 0)) {
            status = 2;
        }
    } else {
        status = 1;
    }

    return status;
}

uint32_t ServoController::enable(bool enable_servo) {
    int32_t status = 0;

    if (is_initialized){
        uint32_t left_status = 0;
        uint32_t right_status = 0;

        if ((enable_servo == true) && (is_enabled == true)) {
            left_status = left_servo.set_angle(DEFAULT_SERVO_ANGLE);
            right_status = right_servo.set_angle(DEFAULT_SERVO_ANGLE);
        } else {
            left_status = left_servo.stop();
            right_status = right_servo.stop();
        }

        if ((left_status == 0) && (right_status == 0)) {
            is_enabled = !is_enabled;
        } else {
            status = 1;
        }
    }

    return status;
}

uint32_t ServoController::get_angle(uint16_t &left_servo_angle, uint16_t &right_servo_angle) {
    uint32_t status = 0;

    if (is_initialized) {
        left_servo.get_angle();
        left_servo_angle = left_servo.get_angle();
        right_servo_angle = right_servo.get_angle();
    } else {
        status = 1;
    }

    return status;
}