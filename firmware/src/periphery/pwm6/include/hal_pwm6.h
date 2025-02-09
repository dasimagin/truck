#pragma once
#ifndef TRUCK_HW_PERIPHERY_PWM6_HAL_PWM6_H_
#define TRUCK_HW_PERIPHERY_PWM6_HAL_PWM6_H_

enum PHASES {
  PHASE_A,
  PHASE_B,
  PHASE_C
};

class PWM6 {
 private:
  PWM6() {};
  ~PWM6() {};
  PWM6(const PWM6 &obj);
  PWM6 &operator=(const PWM6 &obj);

  uint32_t phase_a_channel, phase_b_channel, phase_c_channel;
  uint32_t tim_resolution = 5760;
  uint32_t deadtime = 115; //115

  uint32_t getTimChannel(int channel);

  int getChNumber(int phase);

  void setCompareValue(int channel, uint32_t compareValue);

 public:
  static PWM6 &getInstance() {
      static PWM6 _instance;
      return _instance;
  }

  void set_phase_channels(int a_channel, int b_channel, int c_channel);

  void setSinglePhaseState(PHASES phase, PhaseState state);

  void setPwmDuty(PHASES phase, float duty);

  void setChannels(uint32_t a, uint32_t b, uint32_t c);

  void configure();
};

#endif //TRUCK_HW_PERIPHERY_PWM6_HAL_PWM6_H_
