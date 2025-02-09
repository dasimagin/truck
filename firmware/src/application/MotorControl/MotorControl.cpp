#include "MotorControl.h"

#include "performance_counter.h"
#include "BLDCMotor.h"
#include "hal_gpio.h"

static volatile float speed = 2.0f * 3.1415f * 15;
volatile float iter = 0.05f;
void MotorControl::task() {
    uint32_t start_tick = HAL_GetTick();

    while (true) {
        hal_gpio_set_pin(GPIO_PORT_B, GPIO_PIN_1);
        motor.loopFOC();
        motor.move(speed);
        speed += iter;
        if ((speed > 2.0f * 3.1415f * 40) || (speed < 2.0f * 3.1415f * 15)) {
            iter = -iter;
        }
        printf("%d\n", (int)(speed));
        hal_gpio_reset_pin(GPIO_PORT_B, GPIO_PIN_1);
        vTaskDelay(2);
    }
}

void MotorControl::init() {
    driver.pwm_frequency = 25000;
    motor.voltage_limit = 2;
    driver.voltage_limit = 2;
    driver.voltage_power_supply = 10;
    motor.controller = MotionControlType::velocity;

    driver.init();
    motor.linkDriver(&driver);
    motor.linkSensor(&mag_enc);
    mag_enc.needsSearch();
    BaseType_t status = xTaskCreate(startTaskImpl, "motor_task", 512, this, 4, &task_handle);
    if (status != pdTRUE) {
        printf("Error creating motor task\n");
    }
    vTaskSuspend(task_handle);
}

void MotorControl::calibrate() {
    motor.init();
    motor.initFOC();
    hal_gpio_init(GPIO_PORT_B, GPIO_PIN_1);
    is_initialized = true;
    vTaskResume(task_handle);
}

int32_t MotorControl::set_speed(float speed) {
    int32_t status = 0;
    if (is_initialized) {
        target_speed = speed;
    } else {
        status = 1;
    }
    return status;
}

int32_t MotorControl::enable(bool is_enable) {
    int32_t status = 0;
    if (is_initialized) {
        if (is_motor_enable == is_enable){
            if (is_enable) {
                motor.enable();
            } else {
                motor.disable();
            }
        }
    } else {
        status = 1;
    }
    return status;
}

float MotorControl::get_speed() {
    if (is_initialized) {
        return mag_enc.getVelocity();
    } else {
        return 0.0f;
    }
}
