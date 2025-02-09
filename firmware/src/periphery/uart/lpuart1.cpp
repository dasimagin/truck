#include "lpuart1.h"

#include "stm32g4xx_ll_lpuart.h"
#include "stm32g4xx_ll_gpio.h"
#include "stm32g4xx_ll_bus.h"
#include "stm32g4xx_ll_rcc.h"

extern "C" {
extern int __io_putchar(int ch);
extern int _write(int file, char *ptr, int len);
}

void lpuart1_init() {
    LL_LPUART_InitTypeDef LPUART_InitStruct = {0};
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
    LL_RCC_SetLPUARTClockSource(LL_RCC_LPUART1_CLKSOURCE_PCLK1);
    /* Peripheral clock enable */
    LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_LPUART1);
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
    /* Init GPIO tx */
    GPIO_InitStruct.Pin = LL_GPIO_PIN_2;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_12;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    /* Init LPUART */
    LPUART_InitStruct.PrescalerValue = LL_LPUART_PRESCALER_DIV1;
    LPUART_InitStruct.BaudRate = 500000;
    LPUART_InitStruct.DataWidth = LL_LPUART_DATAWIDTH_8B;
    LPUART_InitStruct.StopBits = LL_LPUART_STOPBITS_1;
    LPUART_InitStruct.Parity = LL_LPUART_PARITY_NONE;
    LPUART_InitStruct.TransferDirection = LL_LPUART_DIRECTION_TX;
    LL_LPUART_Init(LPUART1, &LPUART_InitStruct);
    LL_LPUART_SetTXFIFOThreshold(LPUART1, LL_LPUART_FIFOTHRESHOLD_1_8);
    LL_LPUART_SetRXFIFOThreshold(LPUART1, LL_LPUART_FIFOTHRESHOLD_1_8);
    LL_LPUART_DisableFIFO(LPUART1);
    LL_LPUART_EnableHalfDuplex(LPUART1);
    LL_LPUART_Enable(LPUART1);
    /* Polling LPUART1 initialisation */
    while(!(LL_LPUART_IsActiveFlag_TEACK(LPUART1)))
    {
    }
}

int __io_putchar(int ch) {
    uint8_t c[1];
    c[0] = ch & 0xFF;
    while (!LL_LPUART_IsActiveFlag_TXE_TXFNF(LPUART1)) {
        /* Wait active flag TXE */
    }
    LL_LPUART_TransmitData8(LPUART1, *(uint8_t*)c);
    return ch;
}

int _write(int file, char *ptr, int len) {
    int DataIdx;
    for (DataIdx = 0; DataIdx < len; DataIdx++) {
        __io_putchar(*ptr++);
    }
    return len;
}

void print_buffer(char *buffer, uint32_t size) {
    for (uint32_t i = 0; i < size; ++i) {
        while (!LL_LPUART_IsActiveFlag_TXE_TXFNF(LPUART1)) {
            /* Wait active flag TXE */
        }
        LL_LPUART_TransmitData8(LPUART1, buffer[i]);
    }
}