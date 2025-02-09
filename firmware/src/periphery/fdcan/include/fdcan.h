#ifndef TRUCK_HW_PERIPHERY_FDCAN_FDCAN_H_
#define TRUCK_HW_PERIPHERY_FDCAN_FDCAN_H_
#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_fdcan.h"

extern "C" {
void FDCAN1_IT0_IRQHandler(void);
void FDCAN1_IT1_IRQHandler(void);
}

class FDCAN {
 private:
  FDCAN_HandleTypeDef fdcan_handle;
 public:
  int32_t get_message(uint32_t &message_id, uint8_t *rx_data, uint32_t &rx_data_size);
  void set_rx_complete_callback(void(*callback)());
  static void on_new_rx_data();
  uint32_t transmit(uint32_t message_id, uint8_t *p_data, uint32_t size);
  int32_t init();
};
#endif //TRUCK_HW_PERIPHERY_FDCAN_FDCAN_H_
