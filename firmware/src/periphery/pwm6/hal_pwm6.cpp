#include "stm32g4xx_ll_gpio.h"
#include "stm32g4xx_ll_rcc.h"
#include "stm32g4xx_ll_tim.h"
#include "stm32g4xx_ll_bus.h"
#include "base_classes/BLDCDriver.h"
#include "foc_utils.h"
#include "hal_pwm6.h"

uint32_t PWM6::getTimChannel(int channel) {
    switch (channel) {
        case 1:return LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH1N;
        case 2:return LL_TIM_CHANNEL_CH2 | LL_TIM_CHANNEL_CH2N;
        case 3:return LL_TIM_CHANNEL_CH3 | LL_TIM_CHANNEL_CH3N;
        case 4:return LL_TIM_CHANNEL_CH4 | LL_TIM_CHANNEL_CH4N;
        default:return 0;
    }
}

int PWM6::getChNumber(int phase) {
    switch (phase) {
        case PHASE_A:return phase_a_channel;
        case PHASE_B:return phase_b_channel;
        case PHASE_C:return phase_c_channel;
        default:return -1;
    }
}

void PWM6::setCompareValue(int channel, uint32_t compareValue) {
    switch (channel) {
        case 1:LL_TIM_OC_SetCompareCH1(TIM1, compareValue);
            break;
        case 2:LL_TIM_OC_SetCompareCH2(TIM1, compareValue);
            break;
        case 3:LL_TIM_OC_SetCompareCH3(TIM1, compareValue);
            break;
        case 4:LL_TIM_OC_SetCompareCH4(TIM1, compareValue);
            break;
    }
}

void PWM6::set_phase_channels(int a_channel, int b_channel, int c_channel) {
    phase_a_channel = a_channel;
    phase_b_channel = b_channel;
    phase_c_channel = c_channel;
}

void PWM6::setSinglePhaseState(PHASES phase, PhaseState state) {
    int channel = getChNumber(phase);
    if (channel < 0) {
        return;
    }
    switch (state) {
        case PhaseState::PHASE_OFF:LL_TIM_CC_DisableChannel(TIM1, getTimChannel(channel));
            break;
        default:LL_TIM_CC_EnableChannel(TIM1, getTimChannel(channel));
            break;
    }
}

void PWM6::setPwmDuty(PHASES phase, float duty) {
    int channel = getChNumber(phase);
    int compareValue = _constrain((int) (duty * tim_resolution), 0, tim_resolution);
    setCompareValue(channel, compareValue);
}

void PWM6::setChannels(uint32_t a, uint32_t b, uint32_t c) {
    phase_a_channel = a;
    phase_b_channel = b;
    phase_c_channel = c;
}

void PWM6::configure() {
    LL_TIM_InitTypeDef TIM_InitStruct = {0};
    LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = {0};
    LL_TIM_BDTR_InitTypeDef TIM_BDTRInitStruct = {0};

    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1);

    TIM_InitStruct.Prescaler = 0;
    TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
    TIM_InitStruct.Autoreload = tim_resolution;
    TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
    TIM_InitStruct.RepetitionCounter = 0;
    LL_TIM_Init(TIM1, &TIM_InitStruct);
    LL_TIM_DisableARRPreload(TIM1);
    LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH1);
    TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM1;
    TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
    TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
    TIM_OC_InitStruct.CompareValue = 0;
    TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
    TIM_OC_InitStruct.OCNPolarity = LL_TIM_OCPOLARITY_HIGH;
    TIM_OC_InitStruct.OCIdleState = LL_TIM_OCIDLESTATE_LOW;
    TIM_OC_InitStruct.OCNIdleState = LL_TIM_OCIDLESTATE_LOW;
    LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH1, &TIM_OC_InitStruct);
    LL_TIM_OC_DisableFast(TIM1, LL_TIM_CHANNEL_CH1);
    LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH2);
    LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH2, &TIM_OC_InitStruct);
    LL_TIM_OC_DisableFast(TIM1, LL_TIM_CHANNEL_CH2);
    LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH3);
    LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH3, &TIM_OC_InitStruct);
    LL_TIM_OC_DisableFast(TIM1, LL_TIM_CHANNEL_CH3);
    LL_TIM_SetTriggerOutput(TIM1, LL_TIM_TRGO_RESET);
    LL_TIM_SetTriggerOutput2(TIM1, LL_TIM_TRGO2_RESET);
    LL_TIM_DisableMasterSlaveMode(TIM1);
    TIM_BDTRInitStruct.OSSRState = LL_TIM_OSSR_DISABLE;
    TIM_BDTRInitStruct.OSSIState = LL_TIM_OSSI_DISABLE;
    TIM_BDTRInitStruct.LockLevel = LL_TIM_LOCKLEVEL_OFF;
    TIM_BDTRInitStruct.DeadTime = deadtime;
    TIM_BDTRInitStruct.BreakState = LL_TIM_BREAK_DISABLE;
    TIM_BDTRInitStruct.BreakPolarity = LL_TIM_BREAK_POLARITY_HIGH;
    TIM_BDTRInitStruct.BreakFilter = LL_TIM_BREAK_FILTER_FDIV1;
    TIM_BDTRInitStruct.BreakAFMode = LL_TIM_BREAK_AFMODE_INPUT;
    TIM_BDTRInitStruct.Break2State = LL_TIM_BREAK2_DISABLE;
    TIM_BDTRInitStruct.Break2Polarity = LL_TIM_BREAK2_POLARITY_HIGH;
    TIM_BDTRInitStruct.Break2Filter = LL_TIM_BREAK2_FILTER_FDIV1;
    TIM_BDTRInitStruct.Break2AFMode = LL_TIM_BREAK_AFMODE_INPUT;
    TIM_BDTRInitStruct.AutomaticOutput = LL_TIM_AUTOMATICOUTPUT_DISABLE;
    LL_TIM_BDTR_Init(TIM1, &TIM_BDTRInitStruct);

    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOC);
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
    /* TIM1 GPIO Configuration
    PC0     ------> TIM1_CH1
    PC1     ------> TIM1_CH2
    PC2     ------> TIM1_CH3
    PB13     ------> TIM1_CH1N
    PB14     ------> TIM1_CH2N
    PB15     ------> TIM1_CH3N
    */
    GPIO_InitStruct.Pin = LL_GPIO_PIN_0 | LL_GPIO_PIN_1 | LL_GPIO_PIN_2;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_2;
    LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LL_GPIO_PIN_13 | LL_GPIO_PIN_14;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_6;
    LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LL_GPIO_PIN_15;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_4;
    LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    LL_TIM_CC_EnableChannel(TIM1, getTimChannel(1));
    LL_TIM_CC_EnableChannel(TIM1, getTimChannel(2));
    LL_TIM_CC_EnableChannel(TIM1, getTimChannel(3));
    LL_TIM_EnableCounter(TIM1);
    LL_TIM_EnableAllOutputs(TIM1);
}
