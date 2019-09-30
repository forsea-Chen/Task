/*
 * control.c
 *
 *  Created on: 2019年9月30日
 *      Author: Administrator
 */
#include "main.h"
#include "stm32f4xx_hal_uart.h"
#include "stm32f4xx_hal_dma.h"

//extern uint8_t data_length[255];

//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//  /* Prevent unused argument(s) compilation warning */
//  UNUSED(huart);
//  HAL_UART_Receive_IT(huart,ctrl_data,144);
//  /* NOTE: This function should not be modified, when the callback is needed,
//           the HAL_UART_RxCpltCallback could be implemented in the user file
//   */
//}
void USER_UART_IDLECallback(UART_HandleTypeDef *huart)
    {
	HAL_UART_DMAStop(huart);
	uint8_t data_length=SIZE-__HAL_DMA_GET_COUNTER(&hdma_usart2_rx);
//	HAL_UART_Transmit(&huart2,rx_buff,data_length,0x200);
	data_length=0;
	HAL_UART_Receive_DMA(huart,(uint8_t*)ctrl_data,255);
    }
