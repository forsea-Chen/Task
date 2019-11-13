/*
 * chassis_control.h
 *
 *  Created on: Oct 20, 2019
 *      Author: Administrator
 */

#ifndef CHASSIS_CONTROL_H_
#define CHASSIS_CONTROL_H_

#include "../Middlewares/Tool/Motor.h"
#include "../Middlewares/Tool/MotorControl.h"



#ifdef __cplusplus
extern "C" {
#endif

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

//uint8_t motor_update(CAN_RxHeaderTypeDef *header, uint8_t can_rx_data[]);
void send_solve();
//int32_t CAN_Callback(CAN_RxHeaderTypeDef *header, uint8_t *data);


#ifdef __cplusplus
}
#endif



#endif /* CHASSIS_CONTROL_H_ */
