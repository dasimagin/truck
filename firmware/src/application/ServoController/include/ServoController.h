#ifndef TRUCK_HW_APPLICATION_SERVOCONTROLLER_INCLUDE_SERVOCONTROLLER_H_
#define TRUCK_HW_APPLICATION_SERVOCONTROLLER_INCLUDE_SERVOCONTROLLER_H_

#include "FreeRTOS.h"
#include "task.h"

#include "Servo.h"


class ServoController {
 private:
  bool is_initialized = false;
  bool is_enabled = false;
  TaskHandle_t task_handle;

  Servo &left_servo;
  Servo &right_servo;
  float left_target_angle = 90.0f;
  float right_target_angle = 90.0f;
  void task();

  static void startTaskImpl(void *_this) {
      ((ServoController *) _this)->task();
  }
  ServoController() : left_servo(Servo::get_instance(ServoType::SERVO_LEFT)),
    right_servo(Servo::get_instance(ServoType::SERVO_RIGHT)) {
  };
  ~ServoController() {};
  ServoController(const ServoController &obj) = delete;
  ServoController &operator=(const ServoController &obj) = delete;
 public:
  static ServoController &getInstance();

  uint32_t init();

  uint32_t set_angle(float left_servo_angle, float right_servo_angle);
  uint32_t enable(bool is_enable);

  uint32_t get_angle(uint16_t &left_servo_angle, uint16_t &right_servo_angle);
};
#endif //TRUCK_HW_APPLICATION_SERVOCONTROLLER_INCLUDE_SERVOCONTROLLER_H_
