/*
 * Bsp_freertos.h
 *
 *  Created on: 2019年11月2日
 *      Author: Administrator
 */

#ifndef BSP_FREERTOS_H_
#define BSP_FREERTOS_H_

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "../Application/User/Task_DR16_Process.h"

#ifdef __cplusplus
 extern "C" {
#endif

//extern void DR16_TaskInit();

#define Tiny_Stack_Size       64
#define Small_Stack_Size      128
#define Normal_Stack_Size     256
#define Large_Stack_Size      512
#define Huge_Stack_Size       1024
#define PriorityVeryLow       1
#define PriorityLow           2
#define PriorityBelowNormal   3
#define PriorityNormal        4
#define PriorityAboveNormal   5
#define PriorityHigh          6
#define PrioritySuperHigh     7
#define PriorityRealtime      8
#define SUCCESS   1
#define FAILED    0

#ifdef __cplusplus
}
#endif




#endif /* BSP_FREERTOS_H_ */
