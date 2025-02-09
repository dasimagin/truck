#ifndef TRUCK_HW_PERIPHERY_UART_INCLUDE_LPUART1_H_
#define TRUCK_HW_PERIPHERY_UART_INCLUDE_LPUART1_H_

#include <cstdint>

void lpuart1_init();
void print_buffer(char *buffer, uint32_t len);

#endif // TRUCK_HW_PERIPHERY_UART_INCLUDE_LPUART1_H_