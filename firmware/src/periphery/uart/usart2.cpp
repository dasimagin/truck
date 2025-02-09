#include "usart2.h"

#include "stm32g4xx_ll_usart.h"
#include "stm32g4xx_ll_gpio.h"
#include "stm32g4xx_ll_bus.h"
#include "stm32g4xx_ll_rcc.h"
#include "stm32g4xx_ll_dma.h"

static uint32_t rx_bytes = 0;
static uint8_t *ptr_rx_memory = NULL;
static uint32_t size_of_rx_memory = 0;

extern volatile bool rx_data_available;

extern "C" {
void USART2_IRQHandler(void) {

    if (((LL_USART_IsEnabledIT_RTO(USART2) == true)
        && (LL_USART_IsActiveFlag_RTO(USART2) == true))) {
        LL_USART_ClearFlag_RTO(USART2);
        rx_bytes = size_of_rx_memory - LL_DMA_GetDataLength(DMA2, LL_DMA_CHANNEL_1);

        /* Restart DMA channel for new transfer */
        LL_DMA_DisableChannel(DMA2, LL_DMA_CHANNEL_1);
        LL_DMA_SetDataLength(DMA2, LL_DMA_CHANNEL_1, size_of_rx_memory);
        LL_DMA_EnableChannel(DMA2, LL_DMA_CHANNEL_1);
        rx_data_available = true;
    }

    if (((LL_USART_IsEnabledIT_IDLE(USART2) == true)
        && (LL_USART_IsActiveFlag_IDLE(USART2) == true))) {
        LL_USART_ClearFlag_IDLE(USART2);
    }
    if (LL_USART_IsActiveFlag_ORE(USART2) == true) {
        LL_USART_ClearFlag_ORE(USART2);
    }
}
}

void usart_2_set_memory_for_rx_data(uint8_t *ptr_memory, uint32_t size_of_memory) {
    ptr_rx_memory = ptr_memory;
    size_of_rx_memory = size_of_memory;
    LL_DMA_DisableChannel(DMA2, LL_DMA_CHANNEL_1);
    LL_DMA_SetMemoryAddress(DMA2, LL_DMA_CHANNEL_1, (uint32_t)ptr_rx_memory);
    LL_DMA_SetDataLength(DMA2, LL_DMA_CHANNEL_1, size_of_rx_memory);
    LL_DMA_EnableChannel(DMA2, LL_DMA_CHANNEL_1);
}
/* ====================================================================== */

void usart_2_get_memory_for_rx_data(uint8_t **ptr_memory, uint32_t *ptr_num_of_rx_bytes) {
    *ptr_memory = ptr_rx_memory;
    *ptr_num_of_rx_bytes = rx_bytes;
}

void usart2_init(void) {
    LL_USART_InitTypeDef USART_InitStruct = {0};

    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMAMUX1);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA2);

    LL_RCC_SetUSARTClockSource(LL_RCC_USART2_CLKSOURCE_PCLK1);
    /* Peripheral clock enable */
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);

    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
    /**USART2 GPIO Configuration
     PA15   ------> USART2_RX
    PB3   ------> USART2_TX
    */
    GPIO_InitStruct.Pin = LL_GPIO_PIN_15;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LL_GPIO_PIN_3;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
    LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* USART2 DMA Init */

    /* USART2_RX Init */
    LL_DMA_SetPeriphRequest(DMA2, LL_DMA_CHANNEL_1, LL_DMAMUX_REQ_USART2_RX);
    LL_DMA_SetDataTransferDirection(DMA2, LL_DMA_CHANNEL_1, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
    LL_DMA_SetChannelPriorityLevel(DMA2, LL_DMA_CHANNEL_1, LL_DMA_PRIORITY_LOW);
    LL_DMA_SetMode(DMA2, LL_DMA_CHANNEL_1, LL_DMA_MODE_NORMAL);
    LL_DMA_SetPeriphIncMode(DMA2, LL_DMA_CHANNEL_1, LL_DMA_PERIPH_NOINCREMENT);
    LL_DMA_SetMemoryIncMode(DMA2, LL_DMA_CHANNEL_1, LL_DMA_MEMORY_INCREMENT);
    LL_DMA_SetPeriphSize(DMA2, LL_DMA_CHANNEL_1, LL_DMA_PDATAALIGN_BYTE);
    LL_DMA_SetMemorySize(DMA2, LL_DMA_CHANNEL_1, LL_DMA_MDATAALIGN_BYTE);
    LL_DMA_SetPeriphAddress(DMA2, LL_DMA_CHANNEL_1, (uint32_t)&USART2->RDR);

    USART_InitStruct.PrescalerValue = LL_USART_PRESCALER_DIV1;
    USART_InitStruct.BaudRate = 921600;
    USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
    USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
    USART_InitStruct.Parity = LL_USART_PARITY_NONE;
    USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
    USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
    USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
    LL_USART_Init(USART2, &USART_InitStruct);
    LL_USART_SetTXFIFOThreshold(USART2, LL_USART_FIFOTHRESHOLD_1_8);
    LL_USART_SetRXFIFOThreshold(USART2, LL_USART_FIFOTHRESHOLD_1_8);
    LL_USART_DisableFIFO(USART2);
    LL_USART_ConfigAsyncMode(USART2);

    /* Disable idle interrupt */
    LL_USART_DisableIT_IDLE(USART2);
    /* Configure USART async mode */
    LL_USART_ConfigAsyncMode(USART2);
    /* Enable DMA mode for reception */
    LL_USART_EnableDMAReq_RX(USART2);

    /* Set receiver timeout */
    LL_USART_SetRxTimeout(
        USART2,
        (uint32_t)(
            /* / 1000 - for convert to milliseconds */
            (921600 / 1000)
            * 2 // ms
        )
    );
    /* Set receiver timeout */
    LL_USART_EnableRxTimeout(USART2);
    /* Clear receiver timeout flag */
    LL_USART_ClearFlag_RTO(USART2);
    /* Enable receiver timeout interrupt */
    LL_USART_EnableIT_RTO(USART2);

    /* USART2 interrupt Init */
    NVIC_SetPriority(USART2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
    NVIC_EnableIRQ(USART2_IRQn);

    LL_USART_Enable(USART2);

    /* Polling USART2 initialisation */
    while((!(LL_USART_IsActiveFlag_TEACK(USART2))) || (!(LL_USART_IsActiveFlag_REACK(USART2))))
    {
    }
}

void usart_2_send_buffer(uint8_t *buffer, uint32_t size) {
    for (uint32_t i = 0; i < size; ++i) {
        while (!LL_USART_IsActiveFlag_TXE(USART2)) {
            /* Wait active flag TXE */
        }
        LL_USART_TransmitData8(USART2, buffer[i]);
    }
}