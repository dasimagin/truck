#include "pwm_servo.h"

#include "stm32g4xx_ll_gpio.h"
#include "stm32g4xx_ll_bus.h"

bool PWMServo::is_common_tim_initialized = false;

PWMServo::PWMServo(PWMServoType id) {
    type_ = id;
    switch (id) {
        case PWMServoType::PWM_SERVO_1: {
            timer_channel = LL_TIM_CHANNEL_CH3;
            break;
        }
        case PWMServoType::PWM_SERVO_2: {
            timer_channel = LL_TIM_CHANNEL_CH4;
            break;
        }
    }
}

PWMServo& PWMServo::get_instance(PWMServoType type) {
    static std::unordered_map<PWMServoType, PWMServo *> instances;
    auto it = instances.find(type);
    if (it == instances.end()) {
        instances[type] = new PWMServo(type);
    }
    return *instances[type];
}

uint32_t PWMServo::init() {
    uint32_t status = 0;
    if (!is_initialized) {
        static LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = {0};
        TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM1;
        TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
        TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
        TIM_OC_InitStruct.CompareValue = 0;
        TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;

        if (!is_common_tim_initialized) {
            LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);

            LL_TIM_InitTypeDef TIM_InitStruct = {0};

            LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM3);
            TIM_InitStruct.Prescaler = TIM_PRESCALER;
            TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
            TIM_InitStruct.Autoreload = TIM_AUTORELOAD;
            TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
            LL_TIM_Init(const_cast<TIM_TypeDef*>(common_timer_handle), &TIM_InitStruct);
            LL_TIM_DisableARRPreload(const_cast<TIM_TypeDef*>(common_timer_handle));
            LL_TIM_SetTriggerOutput(const_cast<TIM_TypeDef*>(common_timer_handle), LL_TIM_TRGO_RESET);
            LL_TIM_DisableMasterSlaveMode(const_cast<TIM_TypeDef*>(common_timer_handle));

            is_common_tim_initialized = true;
        }

        LL_TIM_OC_EnablePreload(const_cast<TIM_TypeDef*>(common_timer_handle), timer_channel);
        LL_TIM_OC_Init(const_cast<TIM_TypeDef*>(common_timer_handle), timer_channel, &TIM_OC_InitStruct);
        LL_TIM_OC_DisableFast(const_cast<TIM_TypeDef*>(common_timer_handle), timer_channel);

        LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

        switch (type_) {
            case PWMServoType::PWM_SERVO_1:
                GPIO_InitStruct.Pin = LL_GPIO_PIN_0;
                break;
            case PWMServoType::PWM_SERVO_2:
                GPIO_InitStruct.Pin = LL_GPIO_PIN_1;
                break;
            default:
                status = 1;
                break;
        }

        GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
        GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
        GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
        GPIO_InitStruct.Alternate = LL_GPIO_AF_2;

        if (status == 0) {
            LL_GPIO_Init(GPIOB, &GPIO_InitStruct);
            is_initialized = true;
        }
    }
    return status;
}

uint32_t PWMServo::set_us_impulse(uint32_t us) {
    uint32_t status = 0;
    if (is_initialized){
        if (us < TIM_AUTORELOAD) {
            LL_TIM_CC_EnableChannel(const_cast<TIM_TypeDef*>(common_timer_handle), timer_channel);
            LL_TIM_EnableCounter(const_cast<TIM_TypeDef*>(common_timer_handle));
            switch (timer_channel) {
                case LL_TIM_CHANNEL_CH1: LL_TIM_OC_SetCompareCH1(const_cast<TIM_TypeDef*>(common_timer_handle), us); break;
                case LL_TIM_CHANNEL_CH2: LL_TIM_OC_SetCompareCH2(const_cast<TIM_TypeDef*>(common_timer_handle), us); break;
                case LL_TIM_CHANNEL_CH3: LL_TIM_OC_SetCompareCH3(const_cast<TIM_TypeDef*>(common_timer_handle), us); break;
                case LL_TIM_CHANNEL_CH4: LL_TIM_OC_SetCompareCH4(const_cast<TIM_TypeDef*>(common_timer_handle), us); break;
            }
        } else {
            status = 2;
        }
    } else {
        status = 1;
    }

    return status;
}

uint32_t PWMServo::stop() {
    uint32_t status = 0;
    if (is_initialized) {
        LL_TIM_CC_DisableChannel(const_cast<TIM_TypeDef*>(common_timer_handle), timer_channel);
    } else {
        status = 1;
    }
    return status;
}