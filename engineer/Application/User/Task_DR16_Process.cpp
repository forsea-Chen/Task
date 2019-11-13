/*
 * Task_DR16_Process.cpp
 *
 *  Created on: Nov 1, 2019
 *      Author: Administrator
 */
#include "Task_DR16_Process.h"
TaskHandle_t DR16_Handle;
//DR16_Classdef DR16;
//void Task_DR16_dataprocess(void *argument);
//void DR16_TaskInit();

/*遥控器数据解包任务*/
void Task_DR16_dataprocess(void *argument)
    {
	static USART_COB DR16_data;
	int32_t xLastTick;
	TickType_t xLastWakeTime_DR16;
	xLastWakeTime_DR16 = xTaskGetTickCount();
	for(;;)
	    {
		xLastTick = xTaskGetTickCount();
		while(xQueueReceive(DR16_QueueHandle,&DR16_data,0)==pdTRUE)
		    {

		    }
		vTaskDelayUntil(&xLastWakeTime_DR16,1);
	    }

    }
void DR16_TaskInit()
    {
    xTaskCreate(Task_DR16_dataprocess,"Task_DR16_dataprocess",Small_Stack_Size,NULL,PrioritySuperHigh,&DR16_Handle);
    }

