/**
  ******************************************************************************
  * @file Systemtimer.h
  * @author mannychen (270730634@qq.com)
  * @brief  Header file of Systemtimer.c.
  * @version 0.1
  * @date   2018-10-16
  * @editby

  ******************************************************************************
  * @attention
  *
  * if you had modified this file, please make sure your code does not have many
  * bugs, update the version NO., write dowm your name and the date, the most
  * important is make sure the users will have clear and definite understanding
  * through your new brief.
  ******************************************************************************
  */
#ifndef _SYSTEMTIMER_H_
#define _SYSTEMTIMER_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"


#define micros() Get_SystemTime()
/* Private  ------------------------------------------------------------------*/
/* Exported ------------------------------------------------------------------*/

void systemtimer_init(void);
void systemtimer_callback(TIM_HandleTypeDef *htim);

uint32_t Get_SystemTime(void);
void delay_ms_nos(uint32_t cnt);
void delay_us_nos(uint32_t cnt);

#ifdef __cplusplus
}
#endif
#endif
/************************ COPYRIGHT SCUT-ROBOTLAB *****END OF FILE*************/

