/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
void data_slove();
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
#define SIZE 255
uint8_t dr16_uart_rx_data_handle(UART_HandleTypeDef *huart);
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
uint8_t ctrl_data[255];
int16_t vx,vy,w;
#pragma pack(1)
 typedef struct
 {
 struct
 {
 struct
 {
 uint8_t ch0_h:8; //!< Byte 0

 uint8_t ch0_l:3; //!< Byte 1   ：表示只用3个位
 uint8_t ch1_h:5;

 uint8_t ch1_l:6; //!< Byte 2
 uint8_t ch2_h:2;

 uint8_t ch2_m:8; //!< Byte 3

 uint8_t ch2_l:1; //!< Byte 4
 uint8_t ch3_h:7;

 uint8_t ch3_l:4; //!< Byte 5
 uint8_t s1:2;
 uint8_t s2:2;
 }rc;

 struct
 {
 int16_t x; //!< Byte 6-7
 int16_t y; //!< Byte 8-9
 int16_t z; //!< Byte 10-11
 uint8_t press_l; //!< Byte 12
 uint8_t press_r; //!< Byte 13
 }mouse;

 struct
 {
 uint16_t v; //!< Byte 14-15
 }key;

 uint16_t resv; //!< Byte 16-17
 };
 uint8_t buf[18]; //!< Union --> Byte<0-17>
 }RC_Ctl_Define_t ;
RC_Ctl_Define_t dr16;
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
