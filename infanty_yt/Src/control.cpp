/*
 * control.cpp
 *
 *  Created on: Oct 8, 2019
 *      Author: Administrator
 */
#include "control.h"

int16_t angle;

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
Motor_GM6020 pitch_motor(2);
Motor_GM6020 yaw_motor(1);

MotorAngleCtrl<PID> pitch_motor_ctrl(&pitch_motor);
MotorSpeedCtrl<PID> yaw_motor_ctrl(&yaw_motor);

void head_PID_updata()
    {
    pitch_motor_ctrl.AnglePID.SetPIDParam(5.0f, 0, 0.3f);
    yaw_motor_ctrl.SpeedPID.SetPIDParam(16.0f, 0.02, 0.01f);
    }

void yaw_adjust(float speed)
{
    yaw_motor_ctrl.setTarget(speed);
    yaw_motor_ctrl.Adjust();
  yaw_motor.Out = -yaw_motor.Out;
  MotorMsgSend<Motor_GM6020>(&hcan1, yaw_motor);
}

void picth_adjust(float angle)
{
    pitch_motor_ctrl.setTarget(angle);
    pitch_motor_ctrl.Adjust();
//  head_motor[0].Out = -head_motor[0].Out;
  MotorMsgSend<Motor_GM6020>(&hcan1, pitch_motor);
}

uint8_t head_motor_update(CAN_RxHeaderTypeDef *header, uint8_t can_rx_data[])
{
    if (pitch_motor.CheckID(header->StdId))
	{
	    pitch_motor.update(can_rx_data);
	}
    if (yaw_motor.CheckID(header->StdId))
	{
	    yaw_motor.update(can_rx_data);
	    angle = yaw_motor.getAngle();
	}
  return 0;
}
