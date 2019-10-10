/**
 ******************************************************************************
 * @file Systemtimer.c
 * @author mannychen (270730634@qq.com)
 * @brief  使用定时器，获得精确运行时间，提供堵塞延时接口以及系统运行时间
 * @version 0.1
 * @date date
 * @editby

 ==============================================================================
 ##### How to use this tool #####
 ==============================================================================
 在定时器中断服务函数调用 SystemTimerCnt 进行计数，
 通常我们默认就是使用TIM7作为HAL基准时钟，
 TIM4作为用户系统计时器：用作用户延时函数等
 SYSTick作为FreeRTOS时钟使用

 ******************************************************************************
 * @attention
 *
 * if you had modified this file, please make sure your code does not have many
 * bugs, update the version NO., write dowm your name and the date, the most
 * important is make sure the users will have clear and definite understanding
 * through your new brief.
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "systemtimer.h"

/* Private define ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static volatile uint32_t SystemTimerCnt; //计时的时间是一段有限的时间
extern TIM_HandleTypeDef htim4;

/* Private function prototypes -----------------------------------------------*/

void systemtimer_init(void)
{
  HAL_TIM_Base_Start_IT(&htim4);
  HAL_TIM_Base_Start(&htim4);
}

void systemtimer_callback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM4)
    SystemTimerCnt++;
}

/**
 * @brief  获得系统相对运行时间
 * @param  void
 * @retval 以ms为单位的相对系统时间
 */
uint32_t Get_SystemTime(void)
{
  return TIM4->CNT + SystemTimerCnt * 0xffff;
}

void delay_ms_nos(uint32_t cnt)
{
  uint32_t temp = cnt * 1000 + micros();
  while (temp >= micros());
}

void delay_us_nos(uint32_t cnt)
{
  uint32_t temp = cnt + micros();
  while (temp >= micros());
}
