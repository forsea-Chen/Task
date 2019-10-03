/*
 * control.c
 *
 *  Created on: 2019年9月30日
 *      Author: Administrator
 */

#include "../Drivers/control.h"

#include <stdlib.h>
#include <string.h>
#include <sys/_stdint.h>
#include <main.h>
#include <sys/_stdint.h>
#include "../Drivers/drv_dr16.h"

rc_device_t g_rc_device;

void rc_info_update(rc_device_t *rc_device, uint8_t *buff)
{
  memcpy(&(rc_device->last_rc_info), &(rc_device->rc_info), sizeof(rc_info_t));//把上次的数据赋给last

  rc_info_t *p_rc_info = &(rc_device->rc_info);//建立临时接收结构体，并关联接收结构体
  memcpy(&(rc_device->rc_info), buff, sizeof(rc_info_t));//把buff的数据copy到接收结构体

  p_rc_info->ch1 -= 1024;//地址用->,变量名用。
  p_rc_info->ch2 -= 1024;
  p_rc_info->ch3 -= 1024;
  p_rc_info->ch4 -= 1024;

  p_rc_info->wheel -= 1024;

  /* prevent remote control zero deviation */
  if (p_rc_info->ch1 <= 5 && p_rc_info->ch1 >= -5)
    p_rc_info->ch1 = 0;
  if (p_rc_info->ch2 <= 5 && p_rc_info->ch2 >= -5)
    p_rc_info->ch2 = 0;
  if (p_rc_info->ch3 <= 5 && p_rc_info->ch3 >= -5)
    p_rc_info->ch3 = 0;
  if (p_rc_info->ch4 <= 5 && p_rc_info->ch4 >= -5)
    p_rc_info->ch4 = 0;

  if ((abs(p_rc_info->ch1) > 660) ||
    (abs(p_rc_info->ch2) > 660) ||
    (abs(p_rc_info->ch3) > 660) ||
    (abs(p_rc_info->ch4) > 660))
  {
    memset(&(rc_device->rc_info), 0, sizeof(rc_info_t));
    return;
  }
}
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
//void USER_UART_IDLECallback(UART_HandleTypeDef *huart)
//    {
//	HAL_UART_DMAStop(huart);
//	uint8_t data_length=SIZE-__HAL_DMA_GET_COUNTER(&hdma_usart2_rx);
////	HAL_UART_Transmit(&huart2,rx_buff,data_length,0x200);
//	data_length=0;
//	HAL_UART_Receive_DMA(huart,(uint8_t*)ctrl_data,255);
//    }
 void data_slove()
     {
     rc_info_update(&g_rc_device,dr16_uart_rx_buff);
     vx=(g_rc_device.rc_info.ch3)*2.4;
     vy=(g_rc_device.rc_info.ch4)*2.4;
//     memcpy(&dr16,dr16_uart_rx_buff,sizeof(dr16_uart_rx_buff));
////     dr16.rc.ch0_h=dr16_uart_rx_buff[0];
////     dr16.rc.ch0_l=dr16_uart_rx_buff[1]&0x07;
////     dr16.rc.ch1_h=dr16_uart_rx_buff[1]&0xf8;
////     dr16.rc.ch1_l=dr16_uart_rx_buff[2]&0x3f;
////     dr16.rc.ch2_h=dr16_uart_rx_buff[2]&0xc0;//
////     dr16.rc.ch2_m=dr16_uart_rx_buff[3];
////     dr16.rc.ch2_l=dr16_uart_rx_buff[4]&0x01;//>>7
////     dr16.rc.ch3_h=dr16_uart_rx_buff[4]&0xfe;
////     dr16.rc.ch3_l=dr16_uart_rx_buff[5]&0x0f;
////     dr16.rc.s1=dr16_uart_rx_buff[5]&0x30;
////     dr16.rc.s2=dr16_uart_rx_buff[5]&0xc0;
//     vx=((((dr16_uart_rx_buff[2]&0xc0)<<3)|(dr16_uart_rx_buff[3]<<1)|(dr16_uart_rx_buff[4]&0x01)));
//     vy=((((dr16_uart_rx_buff[4]&0xfe)<<3)|(dr16_uart_rx_buff[5]&0x0f)));
////     w=();
////     dr16.mouse.x=dr16_uart_rx_buff[6]<<8+dr16_uart_rx_buff[7];
////     dr16.mouse.y=dr16_uart_rx_buff[8]<<8+dr16_uart_rx_buff[9];
////     dr16.mouse.z=dr16_uart_rx_buff[10]<<8+dr16_uart_rx_buff[11];
////     dr16.mouse.press_l=dr16_uart_rx_buff[12];
////     dr16.mouse.press_r=dr16_uart_rx_buff[13];
////     dr16.key.v=dr16_uart_rx_buff[14]<<8+dr16_uart_rx_buff[15];
     }
// RC_Ctl_Define_t dr16;

