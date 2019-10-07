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

uint8_t motor_update(CAN_RxHeaderTypeDef *header, uint8_t can_rx_data[]);
void CAN_Transmit();
void move(int16_t vx,int16_t vy,int16_t wx,int16_t wy,uint16_t s1,uint16_t s2);
void PID_set(float p,float i,float d);

#ifdef __cplusplus
}
#endif

#endif /* CONTROL_H_ */

