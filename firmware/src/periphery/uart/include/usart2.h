#ifndef TRUCK_HW_PERIPHERY_UART_INCLUDE_USART2_H_
#define TRUCK_HW_PERIPHERY_UART_INCLUDE_USART2_H_

#include <cstdint>

void usart2_init(void);
void usart_2_set_memory_for_rx_data(uint8_t *ptr_memory, uint32_t size_of_memory);
void usart_2_get_memory_for_rx_data(uint8_t **ptr_memory, uint32_t *ptr_num_of_rx_bytes);
void usart_2_send_buffer(uint8_t *buffer, uint32_t size);

#endif // TRUCK_HW_PERIPHERY_UART_INCLUDE_USART2_H_