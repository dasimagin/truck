#include "hal_gpio.h"

#include "stm32g4xx_ll_gpio.h"
#include "stm32g4xx_ll_bus.h"

static uint32_t get_periph_clock_source(uint32_t gpio_port, uint32_t &clock_source) {
    if (gpio_port == (uint32_t)GPIOA) {
        clock_source = LL_AHB2_GRP1_PERIPH_GPIOA;
    } else if (gpio_port == (uint32_t)GPIOB) {
        clock_source = LL_AHB2_GRP1_PERIPH_GPIOB;
    } else if (gpio_port == (uint32_t)GPIOC) {
        clock_source = LL_AHB2_GRP1_PERIPH_GPIOC;
    } else if (gpio_port == (uint32_t)GPIOD) {
        clock_source = LL_AHB2_GRP1_PERIPH_GPIOD;
    } else {
        return 1;
    }
    return 0;
}

void hal_gpio_set_pin(uint32_t gpio_port, uint32_t gpio_pin) {
    LL_GPIO_SetOutputPin((GPIO_TypeDef*)gpio_port, gpio_pin);
}

void hal_gpio_reset_pin(uint32_t gpio_port, uint32_t gpio_pin) {
    LL_GPIO_ResetOutputPin((GPIO_TypeDef*)gpio_port, gpio_pin);
}

int32_t hal_gpio_init(uint32_t gpio_port, uint32_t gpio_pin) {
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
    uint32_t periph_clock_source;
    if (get_periph_clock_source(gpio_port, periph_clock_source) != 0) {
        return 1;
    }
    LL_AHB2_GRP1_EnableClock(periph_clock_source);
    GPIO_InitStruct.Pin = gpio_pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    return LL_GPIO_Init((GPIO_TypeDef*)gpio_port, &GPIO_InitStruct);
}