#ifndef TRUCK_HW_UTILS_PERFORMANCE_COUNTER_H_
#define TRUCK_HW_UTILS_PERFORMANCE_COUNTER_H_

#include <cstdint>

typedef struct {
  float cpu_frequency_hz;
  uint32_t begin_tick_cnt;
  uint32_t end_tick_cnt;
  float time_ms;
} performance_counter_t;

void performance_counter_init(void);
void performance_counter_code_block_begin(performance_counter_t*);
void performance_counter_code_block_end(performance_counter_t*);
void performance_counter_set_cpu_frequency_hz(performance_counter_t*, float);
uint32_t performance_counter_get_tick_cnt(performance_counter_t*);
float performance_counter_get_time_ms(performance_counter_t*);

#endif //TRUCK_HW_UTILS_PERFORMANCE_COUNTER_H_
