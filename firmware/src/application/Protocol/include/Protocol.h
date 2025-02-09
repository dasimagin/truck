#ifndef TRUCK_HW_APPLICATION_PROTOCOL_INCLUDE_PROTOCOL_H_
#define TRUCK_HW_APPLICATION_PROTOCOL_INCLUDE_PROTOCOL_H_
#include <cstdint>
#include <cstring>
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "fdcan.h"

#define TASK_NOTIFY_RX_DATA_SIGNAL                                         (uint32_t)(0x01)
#define TASK_NOTIFY_INFORM_SIGNAL                                          (uint32_t)(0x02)

#pragma pack(push, 1)
typedef struct {
  float accel_x_axis;
  float accel_y_axis;
  float accel_z_axis;
  float gyro_x_axis;
  float gyro_y_axis;
  float gyro_z_axis;
  float magn_x_axis;
  float magn_y_axis;
  float magn_z_axis;
  bool data_updated;
} protocol_inform_imu_data;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct {
  float motor_velocity;
  float front_left_wheel_velocity;
  float front_right_wheel_velocity;
  bool data_updated;
} protocol_inform_motion_data;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct {
  float target_speed;
  bool is_enable;
} protocol_request_motor;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct {
  int32_t status;
} protocol_response_motor;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct {
  float left_servo_angle;
  float right_servo_angle;
  bool is_enable;
} protocol_request_servo;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct {
  int32_t status;
} protocol_response_servo;
#pragma pack(pop)

#define CALLBACKS_MAX_NUM 20
#define REQUEST_BASE_ID_OFFSET 0
#define INFORM_BASE_ID_OFFSET CALLBACKS_MAX_NUM
#define RESPONSE_BASE_ID_OFFSET (CALLBACKS_MAX_NUM*2)

#define CONFIG_TAG 1
#define MOTOR_TAG 2
#define SERVO_TAG 3

#define IMU_TAG 1
#define MOTION_TAG 2

#define CONFIG_RESPONSE_ID (RESPONSE_BASE_ID_OFFSET + CONFIG_TAG)
#define CONFIG_REQUEST_ID (REQUEST_BASE_ID_OFFSET + CONFIG_TAG)

#define MOTOR_RESPONSE_ID (RESPONSE_BASE_ID_OFFSET + MOTOR_TAG)
#define MOTOR_REQUEST_ID (REQUEST_BASE_ID_OFFSET + MOTOR_TAG)

#define SERVO_RESPONSE_ID (RESPONSE_BASE_ID_OFFSET + SERVO_TAG)
#define SERVO_REQUEST_ID (REQUEST_BASE_ID_OFFSET + SERVO_TAG)

typedef void(*request_callback_t)(const uint8_t *request_data, uint32_t request_data_size,
                                  uint8_t *response_data, uint32_t &response_data_size);

//class Message {
// protected:
//  uint32_t id;
//  uint32_t len = 0;
//  bool is_enable;
//  uint32_t transmit_period_ms;
//  uint32_t last_transmit_ms;
//
// public:
//  bool need_to_transmit() {
//      bool is_needed = false;
//      if (is_enable && (xTaskGetTickCount() - last_transmit_ms) > transmit_period_ms) {
//          is_needed = true;
//      }
//      return is_needed;
//  }
//  virtual int32_t get_message_content(uint8_t *data, float motor_speed) { return -1; };
//  virtual int32_t get_message_content(uint8_t *data, float x, float y, float z) { return -1; };
//  virtual int32_t get_message_content(uint8_t *data) { return -1; };
//  virtual int32_t handle_request(uint8_t* data, uint32_t data_size) { return -1; };
//};
//
//class IMU_Sensor_Message : Message {
// private:
//#pragma pack(push, 1)
//  typedef struct {
//    float x_axis;
//    float y_axis;
//    float z_axis;
//  } protocol_inform_imu;
//#pragma pack(pop)
//
// public:
//  int32_t get_message_content(uint8_t *data, float x, float y, float z) override {
//
//      auto elem = protocol_inform_imu{.x_axis = x, .y_axis = y, .z_axis = z};
//      if (data == nullptr) {
//          return -3;
//      }
//      memcpy(data, &elem, len);
//      return len;
//  }
//};
//
//class Motor_Message : Message {
// private:
//#pragma pack(push, 1)
//  typedef struct {
//    float motor_speed;
//  } protocol_inform_motor;
//#pragma pack(pop)
// public:
//  int32_t get_message_content(uint8_t *data, float motor_speed) override {
//      auto elem = protocol_inform_motor{.motor_speed = motor_speed};
//      if (data == nullptr) {
//          return -2;
//      }
//      memcpy(data, &elem, len);
//      return len;
//  }
//};

//class Config_Request : Message {
// private:
//#pragma pack(push, 1)
//  typedef struct {
//    float motor_speed;
//  } protocol_inform_motor;
//#pragma pack(pop)
// public:
//  int32_t handle_request(uint8_t *data, float motor_speed) override {
//      auto elem = protocol_inform_motor{.motor_speed = motor_speed};
//      if (data == nullptr) {
//          return -2;
//      }
//      memcpy(data, &elem, len);
//      return len;
//  }
//};

class Protocol {
 private:
  FDCAN communication_unit;
  static TaskHandle_t task_handle;
  static TimerHandle_t inform_timer_handle;
  void task();
  request_callback_t request_callback[CALLBACKS_MAX_NUM] = {nullptr};
  static void startTaskImpl(void *_this) {
      ((Protocol *) _this)->task();
  }
  void bind(uint32_t tag, request_callback_t callback);

  void transmit_inform_messages();
  static void send_inform_callback(TimerHandle_t timer_handle) {
      if (timer_handle != NULL) {
          uint32_t task_notify = TASK_NOTIFY_INFORM_SIGNAL;
          BaseType_t task_woken = pdFALSE;
          xTaskNotifyFromISR(task_handle, task_notify, eSetBits, &task_woken);
      }
  }

  static void on_new_request() {
      uint32_t task_notify = TASK_NOTIFY_RX_DATA_SIGNAL;
      BaseType_t task_woken = pdFALSE;
      xTaskNotifyFromISR(task_handle, task_notify, eSetBits, &task_woken);
  }
 public:
  void init();
  void start();
};
#endif //TRUCK_HW_APPLICATION_PROTOCOL_INCLUDE_PROTOCOL_H_
