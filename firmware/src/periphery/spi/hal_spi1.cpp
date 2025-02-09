#include "hal_spi1.h"

#include "FreeRTOS.h"
#include "semphr.h"

#include <cstdio>

void HAL_SPI1::configure() {
    LL_SPI_InitTypeDef SPI_InitStruct = {0};

    SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
    SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
    SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_8BIT;
    SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_HIGH; // high for imu
    SPI_InitStruct.ClockPhase = LL_SPI_PHASE_2EDGE;
    SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
    SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV32; // TODO LL_SPI_BAUDRATEPRESCALER_DIV16
    SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
    SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
    SPI_InitStruct.CRCPoly = 10;
    LL_SPI_Disable(SPI1);
    LL_SPI_Init(SPI1,&SPI_InitStruct);
    LL_SPI_SetStandard(SPI1, LL_SPI_PROTOCOL_MOTOROLA);
    LL_SPI_EnableNSSPulseMgt(SPI1);
    LL_SPI_Enable(SPI1);
}

int32_t HAL_SPI1::exchange_bytes_duplex(
    uint8_t *ptr_tx_bytes,
    size_t tx_size,
    uint8_t *ptr_rx_bytes,
    size_t rx_size
) {
    if (is_communication_started == false) {
        return 1;
    }
    int32_t status = 0;
    uint32_t temp_tick_counter = HAL_GetTick();
    if (LL_SPI_GetTransferDirection(SPI1) != LL_SPI_FULL_DUPLEX){
        LL_SPI_Disable(SPI1);
        LL_SPI_SetTransferDirection(SPI1, LL_SPI_FULL_DUPLEX);
    }
    LL_SPI_Enable(SPI1);
    if (LL_SPI_IsActiveFlag_RXNE(SPI1)) {
        LL_SPI_ReceiveData8(SPI1);
    }
    while ((LL_SPI_IsActiveFlag_TXE(SPI1) == 0) || (LL_SPI_IsActiveFlag_BSY(SPI1) == 1)) {
        if (((HAL_GetTick() - temp_tick_counter)
            > time_out)) {
            status = 1;
            break;
        }
    }

    for (size_t i = 0; i < tx_size; ++i) {
        LL_SPI_TransmitData8(SPI1, ptr_tx_bytes[i]);
        while ((LL_SPI_IsActiveFlag_TXE(SPI1) == 0) || (LL_SPI_IsActiveFlag_BSY(SPI1) == 1)) {
            if (((HAL_GetTick() - temp_tick_counter) > time_out)) {
                status = 1;
                break;
            }
        }
        if (status != 0) {
            break;
        }
//        for (volatile int dumm = 0; dumm <10; ++dumm);
    }

//    for (volatile int dumm = 0; dumm < 100; ++dumm);
    if (LL_SPI_IsActiveFlag_RXNE(SPI1)) {
        LL_SPI_ReceiveData8(SPI1);
    }
    for (size_t i = 0; i < rx_size; ++i) {
        LL_SPI_TransmitData8(SPI1, 0x00);

        while (!(LL_SPI_IsActiveFlag_RXNE(SPI1))) {
            if (((HAL_GetTick() - temp_tick_counter)
                > time_out)) {
                status = 1;
                break;
            }
        }
        if (status != 0) {
            break;
        }
        ptr_rx_bytes[i] = LL_SPI_ReceiveData8(SPI1);
    }
    LL_SPI_Disable(SPI1);
    return status;
}

int32_t HAL_SPI1::exchange_bytes_half_duplex(
    uint8_t *ptr_tx_bytes,
    size_t tx_size,
    uint8_t *ptr_rx_bytes,
    size_t rx_size
) {
    if (is_communication_started == false) {
        return 1;
    }
    int32_t status = 0;
    uint32_t temp_tick_counter = HAL_GetTick(); // TODO Remove in FreeRTOS

    if (LL_SPI_GetTransferDirection(SPI1) != LL_SPI_HALF_DUPLEX_TX){
        LL_SPI_Disable(SPI1);
        LL_SPI_SetTransferDirection(SPI1, LL_SPI_HALF_DUPLEX_TX);
    }

    LL_SPI_Enable(SPI1);
    while ((LL_SPI_IsActiveFlag_TXE(SPI1) == 0) || (LL_SPI_IsActiveFlag_BSY(SPI1) == 1)) {
        if (((HAL_GetTick() - temp_tick_counter)
            > time_out)) {
            status = 1;
            break;
        }
    }

    for (size_t i = 0; i < tx_size; ++i) {
        LL_SPI_TransmitData8(SPI1, ptr_tx_bytes[i]);
        while ((LL_SPI_IsActiveFlag_TXE(SPI1) == 0) || (LL_SPI_IsActiveFlag_BSY(SPI1) == 1)) {
            if (((HAL_GetTick() - temp_tick_counter) > time_out)) {
                status = 1;
                break;
            }
        }
        if (status != 0) {
            break;
        }
    }
    __DSB();
    __ISB();
    __DMB();
    while ((LL_SPI_IsActiveFlag_TXE(SPI1) == 0) || (LL_SPI_IsActiveFlag_BSY(SPI1) == 1)) {
        if (((HAL_GetTick() - temp_tick_counter)
            > time_out)) {
            status = 1;
            break;
        }
    }
    if (status != 0) {
        return status;
    }
    __DSB();
    __ISB();
    __DMB();
    LL_SPI_Disable(SPI1);
    LL_SPI_SetTransferDirection(SPI1, LL_SPI_HALF_DUPLEX_RX);
    LL_SPI_Enable(SPI1);
    uint32_t i = 0;
    while (i < rx_size) {
        if (LL_SPI_IsActiveFlag_RXNE(SPI1) == 1) {
            ptr_rx_bytes[i] = LL_SPI_ReceiveData8(SPI1);
            ++i;
        }
    }
    LL_SPI_Disable(SPI1);
    LL_SPI_SetTransferDirection(SPI1, LL_SPI_HALF_DUPLEX_TX);

    return status;
}

void HAL_SPI1::init() {
    if (is_initialized == false) {
        LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
        LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI1);
        LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
        GPIO_InitStruct.Pin = LL_GPIO_PIN_5 | LL_GPIO_PIN_6;
        GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
        GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
        GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
        GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
        LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = LL_GPIO_PIN_5;
        GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
        GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
        GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
        GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
        LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        semaphore_handle = xSemaphoreCreateMutex();
        if (semaphore_handle == NULL) {
            return;
        }
        configure();
        is_initialized = true;
    }
}

uint32_t get_polarity(enum spi_polarity_ polarity) {
    if (polarity == SPI_POLARITY_LOW) {
        return LL_SPI_POLARITY_LOW;
    } else {
        return LL_SPI_POLARITY_HIGH;
    }
}

int32_t HAL_SPI1::start_communication(enum spi_polarity_ set_polarity) {
    if (is_initialized == false) {
        return 1;
    }

    if (xSemaphoreTake(semaphore_handle, portMAX_DELAY) == pdFALSE) {
        return 2;
    }

    if (LL_SPI_GetClockPolarity(SPI1) != get_polarity(set_polarity)) {
        clock_polarity = set_polarity;
        LL_SPI_Disable(SPI1);
        LL_SPI_SetClockPolarity(SPI1, get_polarity(clock_polarity));
        LL_SPI_Enable(SPI1);
    }

    is_communication_started = true;
    return 0;
}


int32_t HAL_SPI1::stop_communication() {
    if (is_initialized == false) {
        return 1;
    }

    if (xSemaphoreGive(semaphore_handle) == pdFALSE) {
        return 2;
    }
    is_communication_started = false;
    return 0;
}