#include "board.h"

#include "stm32g4xx_ll_rcc.h"
#include "stm32g4xx_ll_utils.h"
#include "stm32g4xx_ll_system.h"
#include "stm32g4xx_ll_pwr.h"
#include "stm32g4xx_ll_tim.h"
#include "stm32g4xx_ll_bus.h"
#include "stm32g4xx_ll_cortex.h"

#include "lpuart1.h"
#include "usart2.h"

static volatile uint32_t system_tick_counter = 0;

extern "C" {
void vApplicationTickHook() {
    LL_TIM_ClearFlag_UPDATE(TIM6);
}

void SysTick_Handler(void) {
	system_tick_counter++;
}
}

uint32_t board_get_tick(void) {
    return system_tick_counter;
}

void board_delay_ms(uint32_t delay_ms) {
    uint32_t timeout_delay_ms = board_get_tick() + delay_ms;
    while (board_get_tick() < timeout_delay_ms);
}

void rtos_tick_timer_init() {
    LL_TIM_InitTypeDef TIM_InitStruct = {0};

    /* Peripheral clock enable */
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM6);

    /* TIM6 interrupt Init */
    NVIC_SetPriority(TIM6_DAC_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
    NVIC_EnableIRQ(TIM6_DAC_IRQn);

    TIM_InitStruct.Prescaler = 144;
    TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
    TIM_InitStruct.Autoreload = 200;
    LL_TIM_Init(TIM6, &TIM_InitStruct);
    LL_TIM_DisableARRPreload(TIM6);
    LL_TIM_SetTriggerOutput(TIM6, LL_TIM_TRGO_UPDATE);
    LL_TIM_DisableMasterSlaveMode(TIM6);
}

void board_start_rtos_timer() {
    LL_TIM_ClearFlag_UPDATE(TIM6);
    LL_TIM_EnableIT_UPDATE(TIM6);
    LL_TIM_EnableCounter(TIM6);
}

extern "C" {
void vPortSetupTimerInterrupt( void ) {
    board_start_rtos_timer();
}
}

static void system_clock_init() {
    LL_FLASH_SetLatency(LL_FLASH_LATENCY_4);
    while(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_4) {};
    LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
    LL_RCC_HSE_Enable();
    /* Wait till HSE is ready */
    while(LL_RCC_HSE_IsReady() != 1) {};

    LL_RCC_HSI_Enable();
    /* Wait till HSI is ready */
    while(LL_RCC_HSI_IsReady() != 1) {};

    LL_RCC_HSI_SetCalibTrimming(64);
    LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_1, 36, LL_RCC_PLLR_DIV_2);
    // TODO Uncomment for USB LL_RCC_PLL_ConfigDomain_48M(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_1, 36, LL_RCC_PLLQ_DIV_6);
    LL_RCC_PLL_EnableDomain_SYS();
    // TODO Uncomment for USB LL_RCC_PLL_EnableDomain_48M();
    LL_RCC_PLL_Enable();
    /* Wait till PLL is ready */
    while(LL_RCC_PLL_IsReady() != 1) {};

    LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
    LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_2);
    /* Wait till System clock is ready */
    while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL) {};

    /* Insure 1us transition state at intermediate medium speed clock*/
    for (volatile uint32_t i = (170 >> 1); i !=0; i--);

    /* Set AHB prescaler*/
    LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
    LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
    LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);

    NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),10, 0));
    NVIC_EnableIRQ(SysTick_IRQn);

    LL_Init1msTick(144000000);
    LL_SetSystemCoreClock(144000000);
    LL_SYSTICK_EnableIT();
}

void board_init() {
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

    NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

    LL_PWR_DisableUCPDDeadBattery();

    system_clock_init();
    rtos_tick_timer_init();
    lpuart1_init();
    usart2_init();
    __enable_irq();
}