/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Includes ------------------------------------------------------------------*/

#include <can.h>
#include <dma.h>
#include <gpio.h>
#include <stm32f405xx.h>
#include <stm32f4xx.h>
#include <stm32f4xx_hal.h>
#include <stm32f4xx_hal_can.h>
#include <stm32f4xx_hal_def.h>
#include <stm32f4xx_hal_flash_ex.h>
#include <stm32f4xx_hal_pwr_ex.h>
#include <stm32f4xx_hal_rcc.h>
#include <stm32f4xx_hal_tim.h>
#include <sys/_stdint.h>
#include <usart.h>
#include "main.h"
#include "control.h"
#include "drv_dr16.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
uint16_t I1,SPEED1,I2,SPEED2,I3,SPEED3,I4,SPEED4;
int16_t dv;
int8_t Rxdata[8], Txdata[8]={0x10,0x10,0x10,0x10,0x10,0x10,0,0};
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
void CAN_Transmit(CAN_HandleTypeDef *hcan,uint32_t Id,uint32_t DLC,int8_t data[]);
void CAN_Receive(CAN_HandleTypeDef *hcan,uint8_t aData[]);
void USER_CAN_ConfigFilter(CAN_HandleTypeDef *hcan);
void motor_moni(int v1,int v2,int v3,int v4);
void motor_run(int ID,int speed);
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//uint8_t dr16_uart_rx_buff[DR16_RX_BUFFER_SIZE];
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
void move(int vx,int vy,int w)
    {
	int v1,v2,v3,v4;
	v1=vy+vx-w*20;
	v2=-(vy-vx+w*20);
	v3=-(vy+vx+w*20);
	v4=vy-vx-w*20;
	motor_moni(v1,v2,v3,v4);
    }
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t rx_buff[255];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int16_t out;
static int16_t error_i=0,error_d=0,error_last=0,error=0;
static float kp=5.5,ki=0.23,kd=0.01;
int16_t PID_OUTPUT(int16_t speed,int16_t target)
    {

	error=target-speed;
	error_i+=error;
	if(error_i>=3000)error_i=3000;
	if(error_i<=-3000)error_i=-3000;
	error_d=error-error_last;
	error_last=error;
	out=kp*error+ki*error_i+kd*error_d;
	if((error>=0&&error<=20)||(error<0&&error>=-20))
	    {out=0;}
	if(out>=10000)out=10000;
	if(out<=-10000)out=-10000;
	return out;
    }

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
//  HAL_UART_Receive_DMA(&huart2,rx_buff,255);
  dr16_uart_init(&huart2);
  USER_CAN_ConfigFilter(&hcan2);
  HAL_CAN_Start(&hcan2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      CAN_Receive(&hcan2, Rxdata);
 //     CAN_Receive2(&hcan2,dr_buff);
      data_slove();
      move(vx,vy,w);
//      data_slove();
//      Txdata[0]=vx>>8;
//      Txdata[1]=vx&0xff;
//      Txdata[2]=vy>>8;
//      Txdata[3]=vy&0xff;
//      Txdata[4]=w>>8;
//      Txdata[5]=w&0xff;
//      CAN_Transmit(&hcan2, 0x100, 8, Txdata);
      HAL_Delay(1);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void USER_CAN_ConfigFilter(CAN_HandleTypeDef *hcan)
      {
      CAN_FilterTypeDef Filter0;
        Filter0.FilterIdHigh = 0;
        Filter0.FilterIdLow = 0;
        Filter0.FilterMaskIdHigh = 0;
        Filter0.FilterMaskIdLow = 0;
        Filter0.FilterFIFOAssignment = CAN_RX_FIFO0;
        Filter0.FilterBank = 0;        //?
        Filter0.FilterMode = CAN_FILTERMODE_IDMASK;
        Filter0.FilterScale = CAN_FILTERSCALE_32BIT;
        Filter0.FilterActivation = ENABLE;
        Filter0.SlaveStartFilterBank = 14;
        HAL_CAN_ConfigFilter(hcan, &Filter0);
//        HAL_CAN_ActivateNotification(hcan,CAN_IT_RX_FIFO0_MSG_PENDING);
      }
void CAN_Transmit(CAN_HandleTypeDef *hcan,uint32_t Id,uint32_t DLC,int8_t data[])
    {
//    uint32_t pTxmailbox;
    CAN_TxHeaderTypeDef Txhead1;
    Txhead1.StdId=Id;
    Txhead1.DLC=DLC;
    Txhead1.IDE=CAN_ID_STD;
    Txhead1.RTR=CAN_RTR_DATA;
    if(HAL_CAN_AddTxMessage(hcan, &Txhead1, data, (uint32_t*)CAN_TX_MAILBOX0)!=HAL_OK)
	{
	    Error_Handler();
	}
    return;
    }
void CAN_Receive(CAN_HandleTypeDef *hcan,uint8_t aData[])
    {
	CAN_RxHeaderTypeDef Rxhead;
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &Rxhead, aData);
	if(Rxhead.StdId==0x201)
	    {
	    SPEED1=((uint16_t)aData[2]<<8)+(uint16_t)aData[3];
//	    I1=((uint16_t)Rxdata[4]<<8)+(uint16_t)Rxdata[5];
	    }
	if(Rxhead.StdId==0x202)
	    {
	    SPEED2=((uint16_t)aData[2]<<8)+(uint16_t)aData[3];
//	    I2=(Rxdata[4]<<8)+Rxdata[5];
	    }
	if(Rxhead.StdId==0x203)
	    {
	    SPEED3=((uint16_t)aData[2]<<8)+(uint16_t)aData[3];
//	    I3=((uint16_t)Rxdata[4]<<8)+Rxdata[5];
	    }
	if(Rxhead.StdId==0x204)
	    {
	    SPEED4=((uint16_t)aData[2]<<8)+(uint16_t)aData[3];
//	    I4=(Rxdata[4]<<8)+Rxdata[5];
	    }
    }
void motor_run(int ID,int speed)
      {
      switch(ID)
	  {
      case 0x201: dv=(int16_t)(PID_OUTPUT(SPEED1,speed));
	      Txdata[0]=dv>>8;
	      Txdata[1]=dv&0XFF;
	      CAN_Transmit(&hcan1,0x200,8,Txdata);
	      break;
      case 0x202: dv=(int16_t)(PID_OUTPUT(SPEED2,speed));
      	      Txdata[2]=dv>>8;
      	      Txdata[3]=dv&0XFF;
      	      CAN_Transmit(&hcan1,0x200,8,Txdata);
      	      break;
      case 0x203: dv=(int16_t)(PID_OUTPUT(SPEED3,speed));
      	      Txdata[4]=dv>>8;
      	      Txdata[5]=dv&0XFF;
      	      CAN_Transmit(&hcan1,0x200,8,Txdata);
      	      break;
      case 0x204: dv=(int16_t)(PID_OUTPUT(SPEED4,speed));
      	      Txdata[6]=dv>>8;
      	      Txdata[7]=dv&0XFF;
      	      CAN_Transmit(&hcan1,0x200,8,Txdata);
      	      break;
	  }
      }
void motor_moni(int v1,int v2,int v3,int v4)
    {
	dv=(uint16_t)(PID_OUTPUT(SPEED1,v1));
	Txdata[0]=dv>>8;
	Txdata[1]=dv&0XFF;
	dv=(uint16_t)(PID_OUTPUT(SPEED1,v2));
	Txdata[2]=dv>>8;
	Txdata[3]=dv&0XFF;
	dv=(uint16_t)(PID_OUTPUT(SPEED1,v3));
	Txdata[4]=dv>>8;
	Txdata[5]=dv&0XFF;
	dv=(uint16_t)(PID_OUTPUT(SPEED1,v4));
	Txdata[6]=dv>>8;
	Txdata[7]=dv&0XFF;
	CAN_Transmit(&hcan2,0x200,8,Txdata);
    }
//void CAN_Receive(CAN_HandleTypeDef *hcan,uint8_t aData[])
//    {
//	CAN_RxHeaderTypeDef Rxhead;
//	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &Rxhead, aData);
//	if(Rxhead.StdId==0x201)
//	    {
//	    SPEED1=((uint16_t)aData[2]<<8)+(uint16_t)aData[3];
////	    I1=((uint16_t)Rxdata[4]<<8)+(uint16_t)Rxdata[5];
//	    }
//	if(Rxhead.StdId==0x202)
//	    {
//	    SPEED2=((uint16_t)aData[2]<<8)+(uint16_t)aData[3];
////	    I2=(Rxdata[4]<<8)+Rxdata[5];
//	    }
//	if(Rxhead.StdId==0x203)
//	    {
//	    SPEED3=((uint16_t)aData[2]<<8)+(uint16_t)aData[3];
////	    I3=((uint16_t)Rxdata[4]<<8)+Rxdata[5];
//	    }
//	if(Rxhead.StdId==0x204)
//	    {
//	    SPEED4=((uint16_t)aData[2]<<8)+(uint16_t)aData[3];
////	    I4=(Rxdata[4]<<8)+Rxdata[5];
//	    }
//    }
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM7 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM7) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
