/*
 * control.cpp
 *
 *  Created on: Oct 6, 2019
 *      Author: Administrator
 */

#include "control.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

Motor_C620 chassis_motor[] = { Motor_C620(1), Motor_C620(2), Motor_C620(3), Motor_C620(4) };
using ChassisCtrl = MotorSpeedCtrl<PID>;
ChassisCtrl chassis_ctrl[] = {
    ChassisCtrl(&chassis_motor[0]),
    ChassisCtrl(&chassis_motor[1]),
    ChassisCtrl(&chassis_motor[2]),
    ChassisCtrl(&chassis_motor[3])
};
//MotorSpeedCtrl<PID> chassis_ctrl[] = {
//				    MotorSpeedCtrl<PID>(&chassis_motor[0]),
//				    MotorSpeedCtrl<PID>(&chassis_motor[1]),
//				    MotorSpeedCtrl<PID>(&chassis_motor[2]),
//				    MotorSpeedCtrl<PID>(&chassis_motor[3])
//				   };
void PID_set(float p,float i,float d)
    {
    for (auto& m : chassis_ctrl)
      {
        m.SpeedPID.SetPIDParam(p, i, d);
        m.SpeedPID.DeadZone = 10.0f;
      }
//	 chassis_ctrl[3].SpeedPID.Kp=p;
//	 chassis_ctrl[3].SpeedPID.Ki=i;
//	 chassis_ctrl[3].SpeedPID.Kd=d;
//	 chassis_ctrl[3].SpeedPID.I_SeparThresh=10000;
//	 chassis_ctrl[3].SpeedPID.DeadZone=20;
//	 chassis_ctrl[0].SpeedPID.Kp=p;
//	 chassis_ctrl[0].SpeedPID.Ki=i;
//	 chassis_ctrl[0].SpeedPID.Kd=d;
//	 chassis_ctrl[0].SpeedPID.I_SeparThresh=10000;
//	 chassis_ctrl[0].SpeedPID.DeadZone=20;
//	 chassis_ctrl[1].SpeedPID.Kp=p;
//	 chassis_ctrl[1].SpeedPID.Ki=i;
//	 chassis_ctrl[1].SpeedPID.Kd=d;
//	 chassis_ctrl[1].SpeedPID.I_SeparThresh=10000;
//	 chassis_ctrl[1].SpeedPID.DeadZone=20;
//	 chassis_ctrl[2].SpeedPID.Kp=p;
//	 chassis_ctrl[2].SpeedPID.Ki=i;
//	 chassis_ctrl[2].SpeedPID.Kd=d;
//	 chassis_ctrl[2].SpeedPID.I_SeparThresh=10000;
//	 chassis_ctrl[2].SpeedPID.DeadZone=20;
    }
void move(int16_t vx,int16_t vy,int16_t wx,int16_t wy,uint16_t s1,uint16_t s2)
    {

	int16_t v1,v2,v3,v4;
	v1=vy+vx-wx*20;
	v2=-(vy-vx+wx*20);
	v3=-(vy+vx+wx*20);
	v4=vy-vx-wx*20;
	if(s1==1)
	    {
		 chassis_ctrl[0].setTarget(-1000);
		 chassis_ctrl[1].setTarget(1000);
		 chassis_ctrl[2].setTarget(1000);
		 chassis_ctrl[3].setTarget(-1000);
	    }
	else
	    {
		 chassis_ctrl[0].setTarget(v1);
		 chassis_ctrl[1].setTarget(v2);
		 chassis_ctrl[2].setTarget(v3);
		 chassis_ctrl[3].setTarget(v4);
	    }
	 chassis_ctrl[0].Adjust();
	 chassis_ctrl[1].Adjust();
	 chassis_ctrl[2].Adjust();
	 chassis_ctrl[3].Adjust();

    }

void CAN_Transmit()
    {
    MotorMsgSend<Motor_C620,4>(&hcan2,chassis_motor);
    }

uint8_t motor_update(CAN_RxHeaderTypeDef *header, uint8_t can_rx_data[])
{
  for (auto& m : chassis_motor)
  {
      if (m.CheckID(header->StdId))
      {
          m.update(can_rx_data);
          return m.ID;
      }
  }
  return 0;
}




//uint8_t motor_update(CAN_RxHeaderTypeDef *header, uint8_t can_rx_data[])
//{
//  for (auto& m : chassis_motor)
//  {
//      if (m.CheckID(header->StdId))
//      {
//          m.update(can_rx_data);
//          return m.ID;
//      }
//  }
//  return 0;
//}

