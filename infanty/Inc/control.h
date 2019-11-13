/*
 * control.h
 *
 *  Created on: Oct 6, 2019
 *      Author: Administrator
 */

#ifndef CONTROL_H_
#define CONTROL_H_

#include <Motor.h>
#include <MotorControl.h>


#ifdef __cplusplus
extern "C" {
#endif

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

//uint8_t motor_update(CAN_RxHeaderTypeDef *header, uint8_t can_rx_data[]);
void CAN_Transmit();
void send_solve();
void move(int16_t vx,int16_t vy,int16_t wx,int16_t wy,uint16_t s1,uint16_t s2);
void PID_SET(float p,float i,float d);
int32_t CAN_Callback(CAN_RxHeaderTypeDef *header, uint8_t *data);
void chassis_adjust();
void chassis_set(int16_t vx,int16_t vy,int16_t w);
#ifdef __cplusplus
}
#endif

#endif /* CONTROL_H_ */

