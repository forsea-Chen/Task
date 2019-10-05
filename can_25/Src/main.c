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
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f4xx_hal_can.h"
#include "stm32f4xx_hal_uart.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
void CAN_Transmit(CAN_HandleTypeDef *hcan,uint32_t Id,uint32_t DLC,uint8_t data[]);
void CAN_Receive(CAN_HandleTypeDef *hcan,uint8_t aData[]);
void USER_CAN_ConfigFilter(CAN_HandleTypeDef *hcan);


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
uint16_t I1,SPEED1,I2,SPEED2,I3,SPEED3,I4,SPEED4;
uint16_t DI=0;
uint8_t Rxdata[8], Txdata[8]={0x10,0x10,0x10,0x10,0x10,0x10,0,0};
uint16_t dv;
uint16_t t=2000;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
//#define Speed 0x10

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CAN1_Init(void);
static void MX_CAN2_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hcan);
  CAN_Receive(hcan, Rxdata);

  /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_CAN_RxFifo0MsgPendingCallback could be implemented in the
            user file
   */
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
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  USER_CAN_ConfigFilter(&hcan1);
  HAL_CAN_Start(&hcan1);
  HAL_CAN_Start(&hcan2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      DI=(uint16_t)(PID_OUTPUT(SPEED1,2000));
 //     I+=DI;
      Txdata[0]=0;
      Txdata[1]=0;
      CAN_Transmit(&hcan1,0x200,8,Txdata);
//      car_run(2000);
//      HAL_Delay(1000);
//      car_back(2000);
//      HAL_Delay(1000);
//      car_leftmove(2000);
//      HAL_Delay(1000);
//      car_rightmove(2000);
//      HAL_Delay(1000);
//      car_leftturn(2000);
//      HAL_Delay(1000);
//      car_rightturn(2000);
//      HAL_Delay(1000);
//      CAN_Receive(&hcan1, Rxdata);
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

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 3;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_9TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief CAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */

  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */

  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 3;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_9TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = DISABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = DISABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */

  /* USER CODE END CAN2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

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
        HAL_CAN_ActivateNotification(hcan,CAN_IT_RX_FIFO0_MSG_PENDING);
      }
void CAN_Transmit(CAN_HandleTypeDef *hcan,uint32_t Id,uint32_t DLC,uint8_t data[])
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
	    SPEED1=((uint16_t)Rxdata[2]<<8)+(uint16_t)Rxdata[3];
//	    I1=((uint16_t)Rxdata[4]<<8)+(uint16_t)Rxdata[5];
	    }
	if(Rxhead.StdId==0x202)
	    {
	    SPEED2=((uint16_t)Rxdata[2]<<8)+(uint16_t)Rxdata[3];
//	    I2=(Rxdata[4]<<8)+Rxdata[5];
	    }
	if(Rxhead.StdId==0x203)
	    {
	    SPEED3=((uint16_t)Rxdata[2]<<8)+(uint16_t)Rxdata[3];
//	    I3=((uint16_t)Rxdata[4]<<8)+Rxdata[5];
	    }
	if(Rxhead.StdId==0x204)
	    {
	    SPEED4=((uint16_t)Rxdata[2]<<8)+(uint16_t)Rxdata[3];
//	    I4=(Rxdata[4]<<8)+Rxdata[5];
	    }
    }
void motor_run(int ID,int speed)
      {
      switch(ID)
	  {
      case 0x201: dv=(uint16_t)(PID_OUTPUT(SPEED1,speed));
	      Txdata[0]=dv>>8;
	      Txdata[1]=dv&0XFF;
	      CAN_Transmit(&hcan1,0x200,8,Txdata);
	      break;
      case 0x202: dv=(uint16_t)(PID_OUTPUT(SPEED2,speed));
      	      Txdata[2]=dv>>8;
      	      Txdata[3]=dv&0XFF;
      	      CAN_Transmit(&hcan1,0x200,8,Txdata);
      	      break;
      case 0x203: dv=(uint16_t)(PID_OUTPUT(SPEED3,speed));
      	      Txdata[4]=dv>>8;
      	      Txdata[5]=dv&0XFF;
      	      CAN_Transmit(&hcan1,0x200,8,Txdata);
      	      break;
      case 0x204: dv=(uint16_t)(PID_OUTPUT(SPEED4,speed));
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
	CAN_Transmit(&hcan1,0x200,8,Txdata);
    }

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
