#include "performance_counter.h"

static volatile uint32_t *DWT_CONTROL   = (uint32_t*)0xE0001000;
static volatile uint32_t *DWT_CYCCNT    = (uint32_t*)0xE0001004;
static volatile uint32_t *DEMCR         = (uint32_t*)0xE000EDFC;
/* ====================================================================== */


void performance_counter_init(void) {
    // Enable the use DWT
    *DEMCR = *DEMCR | 0x01000000;
    // Reset cycle counter
    *DWT_CYCCNT = 0;
    // Enable cycle counter
    *DWT_CONTROL = *DWT_CONTROL | 1 ;
}

void performance_counter_code_block_begin(performance_counter_t *ptr_instance_counter) {
    if (ptr_instance_counter != nullptr) {
        ptr_instance_counter->begin_tick_cnt = *DWT_CYCCNT;
    }
}
/* ====================================================================== */

void performance_counter_code_block_end(performance_counter_t *ptr_instance_counter) {
    if (ptr_instance_counter != nullptr) {
        ptr_instance_counter->end_tick_cnt = *DWT_CYCCNT - ptr_instance_counter->begin_tick_cnt;
        ptr_instance_counter->time_ms = ptr_instance_counter->end_tick_cnt / (
            ptr_instance_counter->cpu_frequency_hz / 1000.0f
        );
    }
}
/* ====================================================================== */

void performance_counter_set_cpu_frequency_hz(performance_counter_t *ptr_instance_counter, float cpu_frequency_hz) {
    if (ptr_instance_counter != nullptr) {
        ptr_instance_counter->cpu_frequency_hz = cpu_frequency_hz;
    }
}
/* ====================================================================== */

uint32_t performance_counter_get_tick_cnt(performance_counter_t *ptr_instance_counter) {
    uint32_t tick_cnt = 0;
    if (ptr_instance_counter != nullptr) {
        tick_cnt = ptr_instance_counter->end_tick_cnt;
    }
    return tick_cnt;
}
/* ====================================================================== */

float performance_counter_get_time_ms(performance_counter_t *ptr_instance_counter) {
    float time_ms = 0;
    if (ptr_instance_counter != nullptr) {
        time_ms = ptr_instance_counter->time_ms;
    }
    return time_ms;
}