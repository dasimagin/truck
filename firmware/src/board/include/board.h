#ifndef TRUCK_HW_BOARD_BOARD_H_
#define TRUCK_HW_BOARD_BOARD_H_

#include "stdint.h"

void board_init();
uint32_t board_get_tick(void);
void board_delay_ms(uint32_t delay_ms);

#endif //TRUCK_HW_BOARD_BOARD_H_
