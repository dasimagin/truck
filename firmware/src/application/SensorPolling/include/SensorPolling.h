#ifndef TRUCK_HW_APPLICATION_SENSORPOLLING_INCLUDE_SENSORPOLLING_H_
#define TRUCK_HW_APPLICATION_SENSORPOLLING_INCLUDE_SENSORPOLLING_H_

#include "FreeRTOS.h"
#include "task.h"

#include "icm20984.h"

class SensorPolling {
 private:
  ICM20984 &imu;
  float gyro_x, gyro_y, gyro_z;
  float accel_x, accel_y, accel_z;
  float magn_x, magn_y, magn_z;
  bool is_initialized = false;
  bool gyro_updated = false;
  bool accel_updated = false;
  bool magn_updated = false;
  TaskHandle_t task_handle;
  void task();

  static void startTaskImpl(void *_this) {
      ((SensorPolling *) _this)->task();
  }
  SensorPolling() : imu(ICM20984::getInstance()) {}
  ~SensorPolling() {}
  SensorPolling(const SensorPolling &obj) = delete;
  SensorPolling &operator=(const SensorPolling &obj) = delete;

 public:
  static SensorPolling &getInstance();

  int32_t init();
  void start();
  int32_t get_gyro_data(float &x_axis, float &y_axis, float &z_axis);
  int32_t get_accel_data(float &x_axis, float &y_axis, float &z_axis);
  int32_t get_magn_data(float &x_axis, float &y_axis, float &z_axis);
};

#endif //TRUCK_HW_APPLICATION_SENSORPOLLING_INCLUDE_SENSORPOLLING_H_
