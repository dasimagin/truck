#include "Protocol.h"
// #include "MotorControl.h"
#include "ServoController.h"
#include "SensorPolling.h"

TaskHandle_t Protocol::task_handle = NULL;
TimerHandle_t Protocol::inform_timer_handle = NULL;

void motor_control_request_handle(const uint8_t *request_data, uint32_t request_data_size,
                                        uint8_t *response_data, uint32_t &response_data_size) {
    if (request_data_size != sizeof(protocol_request_motor)) {
        int32_t status = 0;
        protocol_request_motor request = *(protocol_request_motor *) request_data;
        // MotorControl& MT = MotorControl::getInstance();
        // status |= MT.enable(request.is_enable);
        // status |= MT.set_speed(request.target_speed);
        protocol_response_motor response = {.status = status};
        memcpy(response_data, &response, sizeof(protocol_response_motor));
        response_data_size = sizeof(protocol_response_motor);
    }
}

void servo_control_request_handle(const uint8_t *request_data, uint32_t request_data_size,
                                  uint8_t *response_data, uint32_t &response_data_size) {
    if (request_data_size != sizeof(protocol_request_servo)) {
        int32_t status = 0;
        protocol_request_servo request = *(protocol_request_servo *) request_data;
        ServoController& SC = ServoController::getInstance();
        status |= SC.enable(request.is_enable);
        status |= SC.set_angle(request.left_servo_angle, request.right_servo_angle);
        protocol_response_servo response = {.status = status};
        memcpy(response_data, &response, sizeof(protocol_response_servo));
        response_data_size = sizeof(protocol_response_servo);
    }
}

void Protocol::init() {

    BaseType_t status = xTaskCreate(startTaskImpl, "protocol_task", 512, this, 3, &task_handle);
    if (status == pdTRUE) {
        bind(MOTOR_TAG, motor_control_request_handle);
        bind(SERVO_TAG, servo_control_request_handle);
        inform_timer_handle = xTimerCreate("inform_tim", pdMS_TO_TICKS(20), pdTRUE, NULL, Protocol::send_inform_callback);
        communication_unit.set_rx_complete_callback(Protocol::on_new_request);
        communication_unit.init();
        vTaskResume(task_handle);
        xTimerStart(inform_timer_handle, 10);
    } else {
        printf("Error сreating protocol task\n");
    }
}

void Protocol::task() {
    uint32_t task_notify = 0;
    while (true) {
        if (xTaskNotifyWait(0x00, TASK_NOTIFY_RX_DATA_SIGNAL, &task_notify, portMAX_DELAY) == pdTRUE) {
            if (task_notify == TASK_NOTIFY_RX_DATA_SIGNAL) {
                uint32_t id = 0;
                uint8_t request_data[64];
                uint8_t response_data[64];
                uint32_t request_data_size = 0;
                uint32_t response_data_size = 0;
                communication_unit.get_message(id, request_data, request_data_size);
                int32_t tag = (int32_t) id - REQUEST_BASE_ID_OFFSET;
                if ((tag >= 0)
                    && (tag < CALLBACKS_MAX_NUM)
                    && (request_callback[tag]) != nullptr) {
                    request_callback[tag](request_data, request_data_size, response_data, response_data_size);
                    communication_unit.transmit(tag + RESPONSE_BASE_ID_OFFSET, response_data, response_data_size);
                }
            } else if (task_notify == TASK_NOTIFY_INFORM_SIGNAL) {
                transmit_inform_messages();
            }
            task_notify = 0;
        }
    }
}

void Protocol::bind(uint32_t tag, request_callback_t callback) {
    if (tag < CALLBACKS_MAX_NUM){
        request_callback[tag + REQUEST_BASE_ID_OFFSET] = callback;
    }
}

void Protocol::transmit_inform_messages() {
    protocol_inform_imu_data imu_data;
    protocol_inform_motion_data move_data;
    // static MotorControl &MT = MotorControl::getInstance();
    static SensorPolling &SP = SensorPolling::getInstance();

    int32_t status = 0;
    status |= SP.get_accel_data(imu_data.accel_x_axis, imu_data.accel_y_axis, imu_data.accel_z_axis);
    status |= SP.get_gyro_data(imu_data.gyro_x_axis, imu_data.gyro_y_axis, imu_data.gyro_z_axis);
    status |= SP.get_magn_data(imu_data.magn_x_axis, imu_data.magn_y_axis, imu_data.magn_z_axis);
    imu_data.data_updated = (status == 0);
    communication_unit.transmit(IMU_TAG + INFORM_BASE_ID_OFFSET, (uint8_t *) &imu_data, sizeof(imu_data));

    // move_data.motor_velocity = MT.get_speed();
    move_data.front_left_wheel_velocity = 0.0f;
    move_data.front_right_wheel_velocity = 0.0f;
    move_data.data_updated = true;
    communication_unit.transmit(MOTION_TAG + INFORM_BASE_ID_OFFSET, (uint8_t *) &move_data, sizeof(move_data));
}
