#include "encoder_timer.h"
#include "stm32g4xx_ll_bus.h"
#include "stm32g4xx_ll_tim.h"
#include "stm32g4xx_ll_gpio.h"
#include <vector>
#include <cstdio>

bool EncoderTimer::is_common_tim_initialized = false;

static volatile bool dma_complete_irq[4] = {false};

EncoderTimer& EncoderTimer::get_instance(EncoderType id) {
    static std::unordered_map<EncoderType, EncoderTimer *> instances;
    auto it = instances.find(id);
    if (it == instances.end()) {
        instances[id] = new EncoderTimer(id);
    }
    return *instances[id];
}

uint32_t EncoderTimer::init() {
    raw_buffer.fill(0);

    if (is_common_tim_initialized == false) {
        LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);
        LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
        LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);

        LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMAMUX1);
        LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

        /* DMA interrupt init */
        /* DMA1_Channel1_IRQn interrupt configuration */
        NVIC_SetPriority(DMA1_Channel1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
        NVIC_EnableIRQ(DMA1_Channel1_IRQn);
        /* DMA1_Channel2_IRQn interrupt configuration */
        NVIC_SetPriority(DMA1_Channel2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
        NVIC_EnableIRQ(DMA1_Channel2_IRQn);
        /* DMA1_Channel3_IRQn interrupt configuration */
        NVIC_SetPriority(DMA1_Channel3_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
        NVIC_EnableIRQ(DMA1_Channel3_IRQn);
        /* DMA1_Channel4_IRQn interrupt configuration */
        NVIC_SetPriority(DMA1_Channel4_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
        NVIC_EnableIRQ(DMA1_Channel4_IRQn);
        /* DMAMUX_OVR_IRQn interrupt configuration */
        NVIC_SetPriority(DMAMUX_OVR_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
        NVIC_EnableIRQ(DMAMUX_OVR_IRQn);

        LL_TIM_InitTypeDef TIM_InitStruct = {0};

        TIM_InitStruct.Prescaler = TIM_PRESCALER;
        TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
        TIM_InitStruct.Autoreload = TIM_AUTORELOAD;
        TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
        LL_TIM_Init(TIM2, &TIM_InitStruct);
        LL_TIM_DisableARRPreload(TIM2);
        LL_TIM_SetTriggerOutput(TIM2, LL_TIM_TRGO_RESET);
        LL_TIM_DisableMasterSlaveMode(TIM2);

        LL_TIM_GenerateEvent_UPDATE(TIM2);
        is_common_tim_initialized = true;
    }

    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN; // what??
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    switch (type) {
        case EncoderType::ID_0:
            GPIO_InitStruct.Pin = LL_GPIO_PIN_0;
            GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
            LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
            dma_channel = LL_DMA_CHANNEL_1;
            break;
        case EncoderType::ID_1:
            GPIO_InitStruct.Pin = LL_GPIO_PIN_1;
            GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
            LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
            dma_channel = LL_DMA_CHANNEL_2;
            break;
        case EncoderType::ID_2:
            GPIO_InitStruct.Pin = LL_GPIO_PIN_10;
            GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
            LL_GPIO_Init(GPIOB, &GPIO_InitStruct);
            dma_channel = LL_DMA_CHANNEL_3;
            break;
        case EncoderType::ID_3:
            GPIO_InitStruct.Pin = LL_GPIO_PIN_10;
            GPIO_InitStruct.Alternate = LL_GPIO_AF_10;
            LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
            dma_channel = LL_DMA_CHANNEL_4;
            break;
    }
    LL_DMA_SetDataTransferDirection(const_cast<DMA_TypeDef *>(common_dma_handler), dma_channel, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
    LL_DMA_SetChannelPriorityLevel(const_cast<DMA_TypeDef *>(common_dma_handler), dma_channel, LL_DMA_PRIORITY_LOW);
    LL_DMA_SetMode(const_cast<DMA_TypeDef *>(common_dma_handler), dma_channel, LL_DMA_MODE_CIRCULAR);
    LL_DMA_SetPeriphIncMode(const_cast<DMA_TypeDef *>(common_dma_handler), dma_channel, LL_DMA_PERIPH_NOINCREMENT);
    LL_DMA_SetMemoryIncMode(const_cast<DMA_TypeDef *>(common_dma_handler), dma_channel, LL_DMA_MEMORY_INCREMENT);
    LL_DMA_SetPeriphSize(const_cast<DMA_TypeDef *>(common_dma_handler), dma_channel, LL_DMA_PDATAALIGN_WORD);
    LL_DMA_SetMemorySize(const_cast<DMA_TypeDef *>(common_dma_handler), dma_channel, LL_DMA_MDATAALIGN_WORD);

    LL_TIM_IC_SetActiveInput(const_cast<TIM_TypeDef*>(common_timer_handle), timer_channel, LL_TIM_ACTIVEINPUT_DIRECTTI);
    LL_TIM_IC_SetPrescaler(const_cast<TIM_TypeDef*>(common_timer_handle), timer_channel, LL_TIM_ICPSC_DIV1);
    LL_TIM_IC_SetFilter(const_cast<TIM_TypeDef*>(common_timer_handle), timer_channel, LL_TIM_IC_FILTER_FDIV1);
    LL_TIM_IC_SetPolarity(const_cast<TIM_TypeDef*>(common_timer_handle), timer_channel, LL_TIM_IC_POLARITY_RISING);

    LL_DMA_SetMemoryAddress(const_cast<DMA_TypeDef *>(common_dma_handler), dma_channel, (uint32_t)(raw_buffer.data()));
    switch (type) {
        case EncoderType::ID_0:
            LL_DMA_SetPeriphAddress(const_cast<DMA_TypeDef *>(common_dma_handler), dma_channel, (uint32_t) &(const_cast<TIM_TypeDef *>(common_timer_handle)->CCR1));
            break;
        case EncoderType::ID_1:
            LL_DMA_SetPeriphAddress(const_cast<DMA_TypeDef *>(common_dma_handler), dma_channel, (uint32_t) &(const_cast<TIM_TypeDef *>(common_timer_handle)->CCR2));
            break;
        case EncoderType::ID_2:
            LL_DMA_SetPeriphAddress(const_cast<DMA_TypeDef *>(common_dma_handler), dma_channel, (uint32_t) &(const_cast<TIM_TypeDef *>(common_timer_handle)->CCR3));
            break;
        case EncoderType::ID_3:
            LL_DMA_SetPeriphAddress(const_cast<DMA_TypeDef *>(common_dma_handler), dma_channel, (uint32_t) &(const_cast<TIM_TypeDef *>(common_timer_handle)->CCR4));
            break;
    }
    LL_DMA_SetDataLength(const_cast<DMA_TypeDef *>(common_dma_handler), dma_channel, max_data_size);

    switch (timer_channel) {
        case LL_TIM_CHANNEL_CH1: {
            LL_DMA_SetPeriphRequest(const_cast<DMA_TypeDef *>(common_dma_handler), dma_channel, LL_DMAMUX_REQ_TIM2_CH1);
            LL_TIM_EnableDMAReq_CC1(const_cast<TIM_TypeDef *>(common_timer_handle));
            break;
        }
        case LL_TIM_CHANNEL_CH2: {
            LL_DMA_SetPeriphRequest(const_cast<DMA_TypeDef *>(common_dma_handler), dma_channel, LL_DMAMUX_REQ_TIM2_CH2);
            LL_TIM_EnableDMAReq_CC2(const_cast<TIM_TypeDef *>(common_timer_handle));
            break;
        }
        case LL_TIM_CHANNEL_CH3: {
            LL_DMA_SetPeriphRequest(const_cast<DMA_TypeDef *>(common_dma_handler), dma_channel, LL_DMAMUX_REQ_TIM2_CH3);
            LL_TIM_EnableDMAReq_CC3(const_cast<TIM_TypeDef *>(common_timer_handle));
            break;
        }
        case LL_TIM_CHANNEL_CH4: {
            LL_DMA_SetPeriphRequest(const_cast<DMA_TypeDef *>(common_dma_handler), dma_channel, LL_DMAMUX_REQ_TIM2_CH4);
            LL_TIM_EnableDMAReq_CC4(const_cast<TIM_TypeDef *>(common_timer_handle));
            break;
        }
        default: {
            break;
        }
    }

    LL_DMA_EnableIT_TC(const_cast<DMA_TypeDef *>(common_dma_handler), dma_channel);
    LL_TIM_CC_EnableChannel(const_cast<TIM_TypeDef*>(common_timer_handle), timer_channel);
    LL_DMA_EnableChannel(const_cast<DMA_TypeDef *>(common_dma_handler), dma_channel);
    LL_TIM_EnableCounter(const_cast<TIM_TypeDef*>(common_timer_handle));

    return 0;
}

extern "C" {
void DMA1_Channel1_IRQHandler(void) {
    LL_DMA_ClearFlag_TC1(DMA1);
    dma_complete_irq[0] = true;
}

void DMA1_Channel2_IRQHandler(void) {
    LL_DMA_ClearFlag_TC2(DMA1);
    dma_complete_irq[1] = true;
}

void DMA1_Channel3_IRQHandler(void) {
    LL_DMA_ClearFlag_TC3(DMA1);
    dma_complete_irq[2] = true;
}

void DMA1_Channel4_IRQHandler(void) {
    LL_DMA_ClearFlag_TC4(DMA1);
    dma_complete_irq[3] = true;
}
}

EncoderTimer::EncoderTimer(EncoderType id) : buffer_overwrite(dma_complete_irq[static_cast<int>(id)]) {
    type = id;
    switch (type) {
        case EncoderType::ID_0:
            timer_channel = LL_TIM_CHANNEL_CH1;
            dma_channel = LL_DMA_CHANNEL_1;
            buffer_overwrite = dma_complete_irq[0];
            break;
        case EncoderType::ID_1:
            timer_channel = LL_TIM_CHANNEL_CH2;
            dma_channel = LL_DMA_CHANNEL_1;
            buffer_overwrite = dma_complete_irq[1];
            break;
        case EncoderType::ID_2:
            timer_channel = LL_TIM_CHANNEL_CH3;
            dma_channel = LL_DMA_CHANNEL_1;
            buffer_overwrite = dma_complete_irq[2];
            break;
        case EncoderType::ID_3:
            timer_channel = LL_TIM_CHANNEL_CH4;
            dma_channel = LL_DMA_CHANNEL_1;
            buffer_overwrite = dma_complete_irq[3];
            break;
    }
}


uint32_t EncoderTimer::get_data_size() {
    return max_data_size - LL_DMA_GetDataLength(const_cast<DMA_TypeDef *>(common_dma_handler), dma_channel);
}

uint32_t EncoderTimer::get_data(std::vector<int> &output_data) {
    output_data.clear();
    int buffer_size = 0;
    if (buffer_overwrite) {
        buffer_size = get_data_size() + (raw_buffer.end() - prev_get_it);
        buffer_overwrite = false;
    } else {
        buffer_size = get_data_size() - (prev_get_it - raw_buffer.begin());
    }

    for (int i = 0; i < buffer_size; ++i) {
        output_data.push_back(*prev_get_it - last_cnt_value);
        last_cnt_value = *prev_get_it;
        ++prev_get_it;
        if (prev_get_it == raw_buffer.end()) {
            prev_get_it = raw_buffer.begin();
        }
    }
    return 0;
}


// TODO Why didnt match with declaration

// constexpr float EncoderTimer::get_tick_len_micros() {
//     return (1000000.0f) / (APB1_CLOCK / (TIM_PRESCALER+1));
// }
