/*
 * drv_dr16.h
 *
 *  Created on: Sep 30, 2019
 *      Author: Administrator
 */

#ifndef DRV_DR16_H_
#define DRV_DR16_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

#define DR16_RX_BUFFER_SIZE      (50u)
#define DR16_DATA_LEN            (18u)

void dr16_uart_init(UART_HandleTypeDef *huart);

uint8_t dr16_uart_rx_data_handle(UART_HandleTypeDef *huart);


extern uint8_t dr16_uart_rx_buff[DR16_RX_BUFFER_SIZE];

#ifdef __cplusplus
}
#endif



#endif /* DRV_DR16_H_ */
