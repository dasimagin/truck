#include "stm32g4xx_ll_utils.h"

extern volatile int32_t micros_hundreds;

void _delay(unsigned long ms) {
    LL_mDelay(ms);
}

unsigned long _micros() {
    return  HAL_GetTick() * 1000;
}

void delayMicroseconds(int microsec) {
    return;
}