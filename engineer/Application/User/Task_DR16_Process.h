/*
 * Task_DR16_Process.h
 *
 *  Created on: Nov 1, 2019
 *      Author: Administrator
 */

#include "Bsp_freertos.h"
#include "Queue_Create.h"
//#include "../Middlewares/Tool/Remote_DR16.h"

#ifndef TASK_DR16_PROCESS_H_
#define TASK_DR16_PROCESS_H_

#ifdef __cplusplus
 extern "C" {
#endif

extern TaskHandle_t DR16_Handle;
//extern DR16_Classdef DR16;
void Task_DR16_dataprocess(void *argument);
void DR16_TaskInit();

#ifdef __cplusplus
}
#endif


#endif /* TASK_DR16_PROCESS_H_ */
