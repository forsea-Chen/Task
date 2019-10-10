/*
 * control.h
 *
 *  Created on: Oct 8, 2019
 *      Author: Administrator
 */

#ifndef CONTROL_H_
#define CONTROL_H_

#include <sys/_stdint.h>
#include "main.h"
#include "Motor.h"
#include "MotorControl.h"
#include "PID.h"

#ifdef __cplusplus
extern "C" {
#endif

//int32_t head_Callback(CAN_RxHeaderTypeDef *header, uint8_t can_rx_data[]);
void head_PID_updata();
void yaw_adjust(float angle);
void picth_adjust(float angle);
uint8_t head_motor_update(CAN_RxHeaderTypeDef *header, uint8_t can_rx_data[]);

#ifdef __cplusplus
}
#endif


#endif /* CONTROL_H_ */
