#ifndef TRUCK_HW_PORTSIMPLEFOC_DRIVER_DRIVER6PWM_H_
#define TRUCK_HW_PORTSIMPLEFOC_DRIVER_DRIVER6PWM_H_

#include "base_classes/BLDCDriver.h"
#include "foc_utils.h"
#include "hal_pwm6.h"

class Driver6PWM : public BLDCDriver {
 private:
  int enable_pin;
  // TODO init enable_pin
  float dead_zone;

  PhaseState phase_state[3] = {PhaseState::PHASE_ON, PhaseState::PHASE_ON, PhaseState::PHASE_ON};

  PWM6 &pwm_6_;
 public:
  void writeDutyCycle6PWM(float dc_a, float dc_b, float dc_c);

  Driver6PWM() : pwm_6_(PWM6::getInstance()) {
      pwm_6_.configure();
      pwm_6_.setChannels(1, 2, 3);
  }
  int init();
  void enable();
  void disable();
  void setPwm(float Ua, float Ub, float Uc);
  void setPhaseState(PhaseState sa, PhaseState sb, PhaseState sc);
};
#endif //TRUCK_HW_PORTSIMPLEFOC_DRIVER_DRIVER6PWM_H_
