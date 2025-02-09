#include "base_classes/Sensor.h"
#include "tle5012.h"
#include "foc_utils.h"

#define RAW_TO_RAD(x) (float)((float)(x)/0x7FFF * _2PI)
#define RAW_TO_RAD_PER_SEC(x) (float) (RAW_TO_RAD(x) / 0.0001281)

class MagneticEncoderTLE5012 : public Sensor {
 private:
  TLE5012 &sensor_;

 public:
  MagneticEncoderTLE5012(TLE5012 &sensor) : sensor_(sensor) {

  }

  void init() override {}

  int needsSearch() override {
      return 0;
  }

  int32_t getFullRotations() override {
      return 0;
  }

  double getPreciseAngle() override {
      return angle_prev;
  }

  float getAngle() override {
      return angle_prev;
  }

  float getMechanicalAngle() override {
      return angle_prev;
  }

  void* getAddress() {
      return &sensor_;
  }

  void update() override {
      int32_t raw_angle, raw_speed, raw_revolution;
      if (sensor_.read_angle_value(raw_angle) == 0) {
          angle_prev = RAW_TO_RAD(raw_angle);
      } else {
//            printf("OLD\n");
      }
      if (sensor_.read_angle_speed(raw_speed) == 0) {
          velocity = RAW_TO_RAD_PER_SEC(raw_speed);
      } else {
//          printf("OLD\n");
      }
      full_rotations = 0;
  }

  float getVelocity() override {
      return velocity;
  }
  float getSensorAngle() override {
      return angle_prev;
  }
};