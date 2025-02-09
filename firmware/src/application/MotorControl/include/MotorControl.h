#ifndef TRUCK_HW_APPLICATION_MOTOR_CONTROL_INCLUDE_MOTOR_CONTROL_H_
#define TRUCK_HW_APPLICATION_MOTOR_CONTROL_INCLUDE_MOTOR_CONTROL_H_

#include <memory>
#include "MagneticEncoderTLE5012.cpp"
#include "Driver6PWM.h"
#include "BLDCMotor.h"
#include "FreeRTOS.h"
#include "task.h"

class MotorControl {
 private:
  bool is_initialized;
  BLDCMotor motor;
  Driver6PWM driver;
  MagneticEncoderTLE5012 mag_enc;
  TaskHandle_t task_handle;
  float target_speed;
  bool is_motor_enable;
  void task();

  static void startTaskImpl(void* _this){
      ((MotorControl*)_this)->task();
  }
  MotorControl() : mag_enc(MagneticEncoderTLE5012(TLE5012::getInstance())), motor(BLDCMotor(7)), driver(Driver6PWM()) {}
  ~MotorControl() {};
  MotorControl(const MotorControl &obj) = delete;
  MotorControl &operator=(const MotorControl &obj) = delete;
 public:
  static MotorControl &getInstance() {
      static MotorControl _instance;
      return _instance;
  }
  void init();
  void calibrate();
  int32_t set_speed(float speed);
  float get_speed();
  int32_t enable(bool is_enable);

};
#endif //TRUCK_HW_APPLICATION_MOTOR_CONTROL_INCLUDE_MOTOR_CONTROL_H_
