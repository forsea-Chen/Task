/*
 * control.cpp
 *
 *  Created on: Oct 6, 2019
 *      Author: Administrator
 */

#include "control.h"
#include "cmsis_os.h"


uint16_t speed_send1,speed_send2,speed_send3,speed_send4;
extern SemaphoreHandle_t Binary;

/*Motor_C620 chassis_motor[] = { Motor_C620(1), Motor_C620(2), Motor_C620(3), Motor_C620(4) };
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
	speed_send1=chassis_motor[0].getSpeed();
	speed_send2=chassis_motor[1].getSpeed();
	speed_send3=chassis_motor[2].getSpeed();
	speed_send4=chassis_motor[3].getSpeed();
	int16_t v1,v2,v3,v4;
	v1=vy+vx-wx*20;
	v2=-(vy-vx+wx*20);
	v3=-(vy+vx+wx*20);
	v4=vy-vx-wx*20;
//	if(s1==1)
//	    {
//		 chassis_ctrl[0].setTarget(-1000);
//		 chassis_ctrl[1].setTarget(1000);
//		 chassis_ctrl[2].setTarget(1000);
//		 chassis_ctrl[3].setTarget(-1000);
//	    }
//	else
//	    {
		 chassis_ctrl[0].setTarget(v1);
		 chassis_ctrl[1].setTarget(v2);
		 chassis_ctrl[2].setTarget(v3);
		 chassis_ctrl[3].setTarget(v4);
//	    }
	 chassis_ctrl[0].Adjust();
	 chassis_ctrl[1].Adjust();
	 chassis_ctrl[2].Adjust();
	 chassis_ctrl[3].Adjust();

    }

void CAN_Transmit()
    {
    MotorMsgSend<Motor_C620,4>(&hcan2,chassis_motor);
    }*/

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
Motor_C620 chassis_motor[] = { Motor_C620(1), Motor_C620(2), Motor_C620(3), Motor_C620(4) };
using ChassisCtrl = MotorSpeedCtrl<PID>;
ChassisCtrl chassis_ctrl[] = {
    ChassisCtrl(&chassis_motor[0]),
    ChassisCtrl(&chassis_motor[1]),
    ChassisCtrl(&chassis_motor[2]),
    ChassisCtrl(&chassis_motor[3])
};
template<class SpeedPIDType>
class chassis_control //: public MotorSpeedCtrl<SpeedPIDType>
{

//    Motor_C620 chassis_motor0;
//    Motor_C620 chassis_motor1;
//    Motor_C620 chassis_motor2;
//    Motor_C620 chassis_motor3;
//    MotorSpeedCtrl<SpeedPIDType> chassis_ctrl0;
//    MotorSpeedCtrl<SpeedPIDType> chassis_ctrl1;
//    MotorSpeedCtrl<SpeedPIDType> chassis_ctrl2;
//    MotorSpeedCtrl<SpeedPIDType> chassis_ctrl3;


public:
    chassis_control(){}
    virtual~chassis_control() {}
    virtual void setTarget(int16_t vx,int16_t vy,int16_t w)
	{
	    int16_t v1,v2,v3,v4;
		v1=vy+vx-w*20;
		v2=-(vy-vx+w*20);
		v3=-(vy+vx+w*20);
		v4=vy-vx-w*20;
		 chassis_ctrl[0].setTarget(v1);
		 chassis_ctrl[1].setTarget(v2);
		 chassis_ctrl[2].setTarget(v3);
		 chassis_ctrl[3].setTarget(v4);
	}
    virtual void adjust()
	{
	 chassis_ctrl[0].Adjust();
	 chassis_ctrl[1].Adjust();
	 chassis_ctrl[2].Adjust();
	 chassis_ctrl[3].Adjust();
	 MotorMsgSend<Motor_C620,4>(&hcan2,chassis_motor);
	}
    virtual void PID_set(float p,float i,float d)
        {
        for (auto& m : chassis_ctrl)
          {
            m.SpeedPID.SetPIDParam(p, i, d);
            m.SpeedPID.DeadZone = 10.0f;
            m.SpeedPID.I_SeparThresh=3000;
            m.SpeedPID.LowPass_d_err=0.1;
          }
        }
    virtual uint8_t motor_update(CAN_RxHeaderTypeDef *header, uint8_t can_rx_data[])
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
//protected:

//private:


};
void send_solve()
    {
    speed_send1=chassis_motor[0].getSpeed();
    speed_send2=chassis_motor[1].getSpeed();
    speed_send3=chassis_motor[2].getSpeed();
    speed_send4=chassis_motor[3].getSpeed();
    }

chassis_control<PID> infanty;
void chassis_set(int16_t vx,int16_t vy,int16_t w)
    {
    infanty.setTarget(vx, vy, w);
    }
int32_t CAN_Callback(CAN_RxHeaderTypeDef *header, uint8_t *data)
    {
//	HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, header, data);
	BaseType_t TaskWoken;
	infanty.motor_update(header, data);
	xSemaphoreGiveFromISR(Binary,&TaskWoken);
	return 0;
    }
void PID_SET(float p,float i,float d)
    {
	infanty.PID_set(p, i, d);
    }
void chassis_adjust()
    {
	infanty.adjust();
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

