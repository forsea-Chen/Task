/**
  ******************************************************************************
  * @file   ：GCS_Debug.h
  * @brief  ：GCS_Debug.cpp的头文件。
  * @date   ：2018年12月
  * @author ：华南理工大学机器人实验室（林亮洪）
  ******************************************************************************
  */
#ifndef  _GCS_DEBUG_H_
#define  _GCS_DEBUG_H_
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"

/* Private  ------------------------------------------------------------------*/
/*以下代码用于辅助调试X滑台*/
enum MsgType
{
  Track = 0U,
  Current,
  Actual_Pos,
};
typedef struct
{
  float X_Positive,X_Negative,Y_Positive,Y_Negative,X_Axis,Y_Axis,Z_Axis;
  float Linear_period,Linear_stepWidth,obstacle_period,obstacle_stepWidth,
        transition_stepWidth,rope_stepWidth1,rope_stepWidth2,rope_angle1;
  uint8_t MsgType;
}RoboDebug_Msg;

/* Exported ------------------------------------------------------------------*/ 
union type_change               //数据传输共用体结构
{
  uint8_t   change_u8[4];       // 8位无符号整型【1个字节】
  float     change_float;       //32位浮点型数据【4个字节】
  int       change_int;         //32位有符号整型【4个字节】
  short int change_u16;         //16位有符号整型【2个字节】
};

void GCSDebug_Sent_Set(float *data);
void GCSDebug_Sent_Choose(float * data);
float PARAMETER_Change_float(uint8_t * PARAMETER);
void PARAMETER_MODIFICATION(uint8_t * PARAMETER);
void MODE_MODIFICATION(uint8_t * PARAMETER);
void Sent_Contorl(UART_HandleTypeDef* huart_x);
void RecHandle(uint8_t *data_buf,uint16_t length);

#endif
/************************ COPYRIGHT SCUT-ROBOTLAB *****END OF FILE*************/
