/*
 * dbus.c
 *
 *  Created on: Oct 4, 2019
 *      Author: Administrator
 */


#include "dbus.h"
#include <string.h>
#include <stdlib.h>
#include "stm32f4xx_hal_uart.h"
#include "main.h"
#include "drv_dr16.h"

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
void data_solve()
    {
    rc_info_update(&g_rc_device,dr16_uart_rx_buff);
	vx=(g_rc_device.rc_info.ch3)*2.4;
	vy=(g_rc_device.rc_info.ch4)*2.4;
    }
