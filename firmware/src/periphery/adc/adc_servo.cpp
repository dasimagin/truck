#include "adc_servo.h"

#include "stm32g4xx_ll_adc.h"
#include "stm32g4xx_ll_gpio.h"
#include "stm32g4xx_ll_bus.h"
#include "stm32g4xx_ll_rcc.h"
#include "stm32g4xx_ll_dma.h"
#include "stm32g4xx_ll_utils.h"

ADC& ADC::getInstance() {
    static ADC _instance;
    return _instance;
}

uint32_t ADC::init(void) {
    uint32_t status = 0;
    // TODO should be debug and removed
    volatile uint32_t addr = reinterpret_cast<uint32_t>(adc_handle);

    if (is_initialized == false) {
        LL_ADC_InitTypeDef ADC_InitStruct = {0};
        LL_ADC_REG_InitTypeDef ADC_REG_InitStruct = {0};
        LL_ADC_CommonInitTypeDef ADC_CommonInitStruct = {0};

        LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

        LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMAMUX1);
        LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
        LL_RCC_SetADCClockSource(LL_RCC_ADC12_CLKSOURCE_SYSCLK);

        LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_ADC12);

        LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
        LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
        /*  ADC2 GPIO Configuration
            PA7   ------> ADC2_IN4
            PB11  ------> ADC2_IN14
        */
        GPIO_InitStruct.Pin = LL_GPIO_PIN_7;
        GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
        GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
        LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = LL_GPIO_PIN_11;
        GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
        GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
        LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        LL_DMA_SetPeriphRequest(dma_handle, dma_channel, dma_periph_request);
        LL_DMA_SetDataTransferDirection(dma_handle, dma_channel, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
        LL_DMA_SetChannelPriorityLevel(dma_handle, dma_channel, LL_DMA_PRIORITY_LOW);
        LL_DMA_SetMode(dma_handle, dma_channel, LL_DMA_MODE_CIRCULAR);
        LL_DMA_SetPeriphIncMode(dma_handle, dma_channel, LL_DMA_PERIPH_NOINCREMENT);
        LL_DMA_SetMemoryIncMode(dma_handle, dma_channel, LL_DMA_MEMORY_INCREMENT);
        LL_DMA_SetPeriphSize(dma_handle, dma_channel, LL_DMA_PDATAALIGN_HALFWORD);
        LL_DMA_SetMemorySize(dma_handle, dma_channel, LL_DMA_MDATAALIGN_HALFWORD);
        LL_DMA_SetPeriphAddress(dma_handle, dma_channel, LL_ADC_DMA_GetRegAddr(adc_handle, LL_ADC_DMA_REG_REGULAR_DATA));

        ADC_InitStruct.Resolution = LL_ADC_RESOLUTION_12B;
        ADC_InitStruct.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT;
        ADC_InitStruct.LowPowerMode = LL_ADC_LP_MODE_NONE;
        LL_ADC_Init(adc_handle, &ADC_InitStruct);
        ADC_REG_InitStruct.TriggerSource = LL_ADC_REG_TRIG_SOFTWARE;
        ADC_REG_InitStruct.SequencerLength = LL_ADC_REG_SEQ_SCAN_ENABLE_2RANKS;
        ADC_REG_InitStruct.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE;
        ADC_REG_InitStruct.ContinuousMode = LL_ADC_REG_CONV_CONTINUOUS;
        ADC_REG_InitStruct.DMATransfer = LL_ADC_REG_DMA_TRANSFER_UNLIMITED;
        ADC_REG_InitStruct.Overrun = LL_ADC_REG_OVR_DATA_OVERWRITTEN;
        LL_ADC_REG_Init(adc_handle, &ADC_REG_InitStruct);
        LL_ADC_SetGainCompensation(adc_handle, 0);
        LL_ADC_SetOverSamplingScope(adc_handle, LL_ADC_OVS_DISABLE);
        ADC_CommonInitStruct.CommonClock = LL_ADC_CLOCK_SYNC_PCLK_DIV4;
        LL_ADC_CommonInit(__LL_ADC_COMMON_INSTANCE(adc_handle), &ADC_CommonInitStruct);

        /* Disable ADC deep power down (enabled by default after reset state) */
        LL_ADC_DisableDeepPowerDown(adc_handle);
        /* Enable ADC internal voltage regulator */
        LL_ADC_EnableInternalRegulator(adc_handle);
        /* Delay for ADC internal voltage regulator stabilization. */
        /* Compute number of CPU cycles to wait for, from delay in us. */
        /* Note: Variable divided by 2 to compensate partially */
        /* CPU processing cycles (depends on compilation optimization). */
        /* Note: If system core clock frequency is below 200kHz, wait time */
        /* is only a few CPU processing cycles. */
        uint32_t wait_loop_index;
        wait_loop_index = ((LL_ADC_DELAY_INTERNAL_REGUL_STAB_US * (144000000 / (100000 * 2))) / 10);
        while(wait_loop_index != 0)
        {
            wait_loop_index--;
        }

        LL_ADC_REG_SetSequencerRanks(adc_handle, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_4);
        // TODO Test different sampling time
        LL_ADC_SetChannelSamplingTime(adc_handle, LL_ADC_CHANNEL_4, LL_ADC_SAMPLINGTIME_6CYCLES_5);
        LL_ADC_SetChannelSingleDiff(adc_handle, LL_ADC_CHANNEL_4, LL_ADC_SINGLE_ENDED);

        LL_ADC_REG_SetSequencerRanks(adc_handle, LL_ADC_REG_RANK_2, LL_ADC_CHANNEL_14);
        LL_ADC_SetChannelSamplingTime(adc_handle, LL_ADC_CHANNEL_14, LL_ADC_SAMPLINGTIME_6CYCLES_5);
        LL_ADC_SetChannelSingleDiff(adc_handle, LL_ADC_CHANNEL_14, LL_ADC_SINGLE_ENDED);

        LL_ADC_SetSamplingTimeCommonConfig(adc_handle, LL_ADC_SAMPLINGTIME_COMMON_DEFAULT);

        LL_ADC_Enable(adc_handle);

        LL_ADC_StartCalibration(adc_handle, LL_ADC_SINGLE_ENDED);
        while (LL_ADC_IsCalibrationOnGoing(adc_handle) != 0) {}
        for(volatile int i = 0; i < 10000; ++i );

        LL_ADC_Enable(adc_handle);

        for(volatile int i = 0; i < 10000; ++i );
        while (LL_ADC_IsActiveFlag_ADRDY(adc_handle) == 0) {}

        is_initialized = true;
    }

    return status;
}

uint32_t ADC::start_measurment() {
    uint32_t status = 0;
    if (is_initialized == true) {
        if (is_started == false) {
            data[0] = 0;
            data[1] = 0;
            LL_DMA_SetDataLength(dma_handle, dma_channel, 2);
            LL_DMA_SetMemoryAddress(dma_handle, dma_channel, (uint32_t)data);
            LL_DMA_EnableChannel(dma_handle, dma_channel);
            LL_ADC_REG_StartConversion(adc_handle);

            is_started = true;
        } else {
            status = 2;
        }
    } else {
        status = 1;
    }

    return status;
}

uint32_t ADC::stop_measurment() {
    uint32_t status = 0;
    if (is_initialized == true) {
        LL_ADC_REG_StopConversion(adc_handle);
        LL_DMA_DisableChannel(dma_handle, dma_channel);

        is_started = false;
    } else {
        status = 1;
    }

    return status;
}

uint32_t ADC::get_value(ADCServoType type, uint16_t &value) {
    uint32_t status = 0;

    if (is_initialized == true) {
        if (is_started == true) {
            switch (type) {
                case ADCServoType::ADC_SERVO_1:
                    value = data[0];
                    break;
                case ADCServoType::ADC_SERVO_2:
                    value = data[1];
                    break;
                default:
                    value = 0;
                    status = 3;
                    break;
            }
        } else {
            status = 2;
        }
    } else {
        status = 1;
    }

    return status;
}