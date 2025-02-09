#include "fdcan.h"
#include "stm32g4xx_hal_fdcan.h"
#include "stm32g4xx_ll_gpio.h"
#include "stm32g4xx_ll_bus.h"
#include "stm32g4xx_ll_rcc.h"

#include <cstdio>

FDCAN_HandleTypeDef *fdcan1;

void(*rx_new_data_callback)() = nullptr;

extern "C" {
void FDCAN1_IT0_IRQHandler(void) {
    HAL_FDCAN_IRQHandler(fdcan1);
}

void FDCAN1_IT1_IRQHandler(void) {
    HAL_FDCAN_IRQHandler(fdcan1);
}

FDCAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[64];

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0){
        if (rx_new_data_callback != nullptr) {
            rx_new_data_callback();
        }
    }
}

}


int32_t FDCAN::get_message(uint32_t &message_id, uint8_t *rx_data, uint32_t &rx_data_size) {
    uint32_t rx_fifo0_size = HAL_FDCAN_GetRxFifoFillLevel(&fdcan_handle, FDCAN_RX_FIFO0);
    if (rx_fifo0_size > 0) {
        return 1;
    }
    FDCAN_RxHeaderTypeDef rx_header;

    if (HAL_FDCAN_GetRxMessage(&fdcan_handle, FDCAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK) {
        return 2;
    }

    message_id = rx_header.Identifier;
    rx_data_size = rx_header.DataLength;

    return 0;
}

int32_t FDCAN :: init() {
    int32_t status = 0;
    fdcan1 = &fdcan_handle;
    LL_RCC_SetFDCANClockSource(LL_RCC_FDCAN_CLKSOURCE_PCLK1);
    LL_APB1_GRP1_EnableClock(RCC_APB1ENR1_FDCANEN); // SET_BIT(RCC->APB1ENR1, RCC_APB1ENR1_FDCANEN);
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB); //  SET_BIT(RCC->AHB2ENR, RCC_AHB2ENR_GPIOBEN);

    HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
//    HAL_NVIC_SetPriority(FDCAN1_IT1_IRQn, 0, 0);
//    HAL_NVIC_EnableIRQ(FDCAN1_IT1_IRQn);

    LL_GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pin = LL_GPIO_PIN_8|LL_GPIO_PIN_9;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_9;
    LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    fdcan_handle.Instance = FDCAN1;
    fdcan_handle.Init.ClockDivider = FDCAN_CLOCK_DIV1;
    fdcan_handle.Init.FrameFormat = FDCAN_FRAME_FD_BRS;
    fdcan_handle.Init.Mode = FDCAN_MODE_EXTERNAL_LOOPBACK; //FDCAN_MODE_EXTERNAL_LOOPBACK
    fdcan_handle.Init.AutoRetransmission = DISABLE;
    fdcan_handle.Init.TransmitPause = ENABLE;
    fdcan_handle.Init.ProtocolException = DISABLE;
    fdcan_handle.Init.NominalPrescaler = 4;
    fdcan_handle.Init.NominalSyncJumpWidth = 9;
    fdcan_handle.Init.NominalTimeSeg1 = 26;
    fdcan_handle.Init.NominalTimeSeg2 = 9;
    fdcan_handle.Init.DataPrescaler = 1;
    fdcan_handle.Init.DataSyncJumpWidth = 9;
    fdcan_handle.Init.DataTimeSeg1 = 26;
    fdcan_handle.Init.DataTimeSeg2 = 9;
    fdcan_handle.Init.StdFiltersNbr = 1;
    fdcan_handle.Init.ExtFiltersNbr = 1;
    fdcan_handle.Init.TxFifoQueueMode = FDCAN_TX_QUEUE_OPERATION;

    if (HAL_FDCAN_Init(&fdcan_handle) != HAL_OK)
    {
        status = 1;
    }

    if (HAL_FDCAN_Start(&fdcan_handle) != HAL_OK)
    {
        status = 2;
    }

    if (HAL_FDCAN_ActivateNotification(&fdcan_handle, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
    {
        status = 3;
    }
    return status;
}

uint32_t FDCAN :: transmit(uint32_t message_id, uint8_t *p_data, uint32_t size){
    FDCAN_TxHeaderTypeDef tx_header;
    tx_header.Identifier = message_id;
    tx_header.IdType = FDCAN_STANDARD_ID;
    tx_header.TxFrameType = FDCAN_DATA_FRAME;
    tx_header.DataLength = FDCAN_DLC_BYTES_32;
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch = FDCAN_BRS_ON;
    tx_header.FDFormat = FDCAN_FD_CAN;
    tx_header.TxEventFifoControl = FDCAN_STORE_TX_EVENTS;
    tx_header.MessageMarker = 0x52;
    uint32_t status = 0;
    status |= HAL_FDCAN_AddMessageToTxFifoQ(&fdcan_handle, &tx_header, p_data);

    return status;
}

void FDCAN::set_rx_complete_callback(void (*callback)()) {
    rx_new_data_callback = callback;
}

void FDCAN::on_new_rx_data() {
    printf("from class\n");
}
