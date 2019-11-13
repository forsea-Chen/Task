/*
 * control.cpp
 *
 *  Created on: Oct 8, 2019
 *      Author: Administrator
 */
#include "control.h"

int16_t angle,angle2;

float yaw_angle;
float pitch_angle;
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
Motor_GM6020 pitch_motor(2);
Motor_GM6020 yaw_motor(1);

MotorAngleCtrl<PID> pitch_motor_ctrl(&pitch_motor);
MotorAngleCtrl<PID> yaw_motor_ctrl(&yaw_motor);

void head_PID_updata()
    {
    pitch_motor_ctrl.AnglePID.SetPIDParam(200.0f, 0.02f, 0.4f);
    yaw_motor_ctrl.AnglePID.SetPIDParam(200.0f, 0.02f, 0.3f);
    }

void yaw_adjust(float angle)
{
//    static float yaw_angle;
    yaw_angle += angle;
//    angle = angle + yaw_motor.getAngle();
    yaw_motor_ctrl.setTarget(yaw_angle);
    yaw_motor_ctrl.Adjust();
    yaw_motor.Out = -yaw_motor.Out;
    MotorMsgSend<Motor_GM6020>(&hcan1, yaw_motor);
}

void picth_adjust(float angle)
{
//    static float pitch_angle;
    pitch_angle += angle;
//    angle = angle + pitch_motor.getAngle();
    if(pitch_angle>=20) pitch_angle=20;
    if(pitch_angle<=-10) pitch_angle=-10;
    pitch_motor_ctrl.setTarget(pitch_angle);
    pitch_motor_ctrl.Adjust();
//  head_motor[0].Out = -head_motor[0].Out;
    MotorMsgSend<Motor_GM6020>(&hcan1, pitch_motor);
}

int32_t head_motor_update(CAN_RxHeaderTypeDef *header, uint8_t can_rx_data[])
{
    if (pitch_motor.CheckID(header->StdId))
	{
	    pitch_motor.update(can_rx_data);
	}
    if (yaw_motor.CheckID(header->StdId))
	{
	    yaw_motor.update(can_rx_data);
	    angle = yaw_motor.getAngle();
	    angle2 = pitch_motor.getAngle();
	}
  return 0;
}
