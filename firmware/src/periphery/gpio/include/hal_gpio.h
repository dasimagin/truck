#include <cstdint>
#include "stm32g431xx.h"

#pragma once

#define GPIO_PORT_A                                     ((uint32_t)GPIOA)
#define GPIO_PORT_B                                     ((uint32_t)GPIOB)
#define GPIO_PORT_C                                     ((uint32_t)GPIOC)
/* ====================================================================== */

// #define GPIO_PIN_0                                          LL_GPIO_PIN_0
// #define GPIO_PIN_1                                          LL_GPIO_PIN_1
// #define GPIO_PIN_2                                          LL_GPIO_PIN_2
// #define GPIO_PIN_3                                          LL_GPIO_PIN_3
// #define GPIO_PIN_4                                          LL_GPIO_PIN_4
// #define GPIO_PIN_5                                          LL_GPIO_PIN_5
// #define GPIO_PIN_6                                          LL_GPIO_PIN_6
// #define GPIO_PIN_7                                          LL_GPIO_PIN_7
// #define GPIO_PIN_8                                          LL_GPIO_PIN_8
// #define GPIO_PIN_9                                          LL_GPIO_PIN_9
// #define GPIO_PIN_10                                         LL_GPIO_PIN_10
// #define GPIO_PIN_11                                         LL_GPIO_PIN_11
// #define GPIO_PIN_12                                         LL_GPIO_PIN_12
// #define GPIO_PIN_13                                         LL_GPIO_PIN_13
// #define GPIO_PIN_14                                         LL_GPIO_PIN_14
// #define GPIO_PIN_15                                         LL_GPIO_PIN_15
/* ====================================================================== */

int32_t hal_gpio_init(uint32_t, uint32_t);

void hal_gpio_set_pin(uint32_t, uint32_t);

void hal_gpio_reset_pin(uint32_t, uint32_t);
