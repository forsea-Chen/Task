/*
 * Queue_Create.h
 *
 *  Created on: Nov 1, 2019
 *      Author: Administrator
 */

#ifndef QUEUE_CREATE_H_
#define QUEUE_CREATE_H_

#include "FreeRTOS.h"
#include "queue.h"
#include "cmsis_os.h"

#ifdef __cplusplus
 extern "C" {
#endif

extern QueueHandle_t  DR16_QueueHandle;
extern QueueHandle_t  Motor_QueueHandle;

/*Uart数据包结构体，用于队列传输*/
 typedef struct
 {
   uint8_t  port_num;
   int16_t  len;
   void*    address;
 }USART_COB;

/*CAN数据包结构体，用于队列传输*/
 typedef struct{
   uint16_t  ID;
   uint8_t   DLC;
   uint8_t   Data[8];
 }COB_TypeDef;

#ifdef __cplusplus
}
#endif




#endif /* QUEUE_CREATE_H_ */
