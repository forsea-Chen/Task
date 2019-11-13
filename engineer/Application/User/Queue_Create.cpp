/*
 * Queue_Create.cpp
 *
 *  Created on: Nov 1, 2019
 *      Author: Administrator
 */
#include "Queue_Create.h"
/*创建所有队列句柄*/
QueueHandle_t  DR16_QueueHandle;
QueueHandle_t  Motor_QueueHandle;

/*队列初始化,创建所有队列*/
void Queue_Init()
    {
    DR16_QueueHandle = xQueueCreate(2,sizeof(USART_COB));
    DR16_QueueHandle = xQueueCreate(10,sizeof(COB_TypeDef));
    }



