#include "base_classes/BLDCDriver.h"
#include "foc_utils.h"
#include "hal_pwm6.h"
#include "Driver6PWM.h"

void Driver6PWM::writeDutyCycle6PWM(float dc_a, float dc_b, float dc_c) {
    // phase a
    pwm_6_.setSinglePhaseState(PHASE_A, phase_state[0]);
    if (phase_state[0] != PhaseState::PHASE_OFF) {
        pwm_6_.setPwmDuty(PHASE_A, dc_a);
    }
    // phase b
    pwm_6_.setSinglePhaseState(PHASE_B, phase_state[1]);
    if (phase_state[1] != PhaseState::PHASE_OFF) {
        pwm_6_.setPwmDuty(PHASE_B, dc_b);
    }
    // phase c
    pwm_6_.setSinglePhaseState(PHASE_C, phase_state[2]);
    if (phase_state[2] != PhaseState::PHASE_OFF) {
        pwm_6_.setPwmDuty(PHASE_C, dc_c);
    }
}

int Driver6PWM::init() {
    initialized = true;
    return 0;
}

void Driver6PWM::enable() {
    // enable_pin the driver - if enable_pin pin available
    if (_isset(enable_pin) == true) {
        //digitalWrite(enable_pin, enable_active_high);
    }
    // set phase state enabled
    setPhaseState(PhaseState::PHASE_ON, PhaseState::PHASE_ON, PhaseState::PHASE_ON);
    // set zero to PWM
    setPwm(0, 0, 0);

}

void Driver6PWM::disable() {
    // set phase state to disabled
    setPhaseState(PhaseState::PHASE_OFF, PhaseState::PHASE_OFF, PhaseState::PHASE_OFF);
    // set zero to PWM
    setPwm(0, 0, 0);
    // disable the driver - if enable_pin pin available
    if (_isset(enable_pin) == true) {
        //digitalWrite(enable_pin, !enable_active_high);
    }
}

void Driver6PWM::setPwm(float Ua, float Ub, float Uc) {
    // limit the voltage in driver
    Ua = _constrain(Ua, 0, voltage_limit);
    Ub = _constrain(Ub, 0, voltage_limit);
    Uc = _constrain(Uc, 0, voltage_limit);
    // calculate duty cycle in [0,1]
    dc_a = _constrain(Ua / voltage_power_supply, 0.0f, 1.0f);
    dc_b = _constrain(Ub / voltage_power_supply, 0.0f, 1.0f);
    dc_c = _constrain(Uc / voltage_power_supply, 0.0f, 1.0f);

    writeDutyCycle6PWM(dc_a, dc_b, dc_c);
}

void Driver6PWM::setPhaseState(PhaseState sa, PhaseState sb, PhaseState sc) {
    phase_state[0] = sa;
    phase_state[1] = sb;
    phase_state[2] = sc;
}
