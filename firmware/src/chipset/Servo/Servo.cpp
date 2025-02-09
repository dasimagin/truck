#include "Servo.h"

#include "board.h"

Servo& Servo::get_instance(ServoType type) {
    static std::unordered_map<ServoType, Servo *> instances;
    auto it = instances.find(type);
    if (it == instances.end()) {
        instances[type] = new Servo(type);
    }
    return *instances[type];
}

uint32_t Servo::init() {
    uint32_t status = 0;

    switch (type_) {
        case ServoType::SERVO_LEFT:
            max_angle = 270.0f; //135.0f + 48.0f + 1.0f;
            home_angle = 98.0f;
            min_angle = 0.0f; //135.0f - 62.0f - 1.0f;
            break;
        case ServoType::SERVO_RIGHT:
            max_angle = 270.0f; //135.0f + 62.0f + 1.0f;
            home_angle = 172.0f;
            min_angle = 0.0f; // 135.0f - 48.0f - 1.0f;
            break;
    }

    if (pwm.init() == 0) {
        if (adc.init() == 0) {
            is_initialized = true;
        } else {
            status = 2;
        }
    } else {
        status = 1;
    }

    return status;
}

uint32_t Servo::calibrate(void) {
    set_angle(home_angle);
    return 0;
}

uint32_t Servo::set_angle(float angle) {
    uint32_t status = 0;

    if (is_initialized == true) {
        if ((angle <= max_angle) && (angle >= min_angle)) {
            uint32_t us = 500 + (uint32_t) ((2500.0f - 500.0f) / 270.0f * angle);
            pwm.set_us_impulse(us);
        } else {
            status = 2;
        }
    } else {
        status = 1;
    }

    return status;
}

uint16_t Servo::get_angle() {
    uint16_t raw_value = 0;

    if (adc.read_value(raw_value) != 0) {
        raw_value = 0;
    }

    return raw_value;
}

uint32_t Servo::stop() {
    uint32_t status = 0;

    if (is_initialized) {
        pwm.stop();
    } else {
        status = 1;
    }

    return status;
}
