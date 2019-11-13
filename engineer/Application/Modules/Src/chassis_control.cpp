/*
 * chassis_control.cpp
 *
 *  Created on: Oct 20, 2019
 *      Author: Administrator
 */

#include "../Inc/chassis_control.h"

#include <stm32f4xx_hal_can.h>
#include <sys/_stdint.h>

#include "../../../Middlewares/Tool/PID.h"

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
//    speed_send1=chassis_motor[0].getSpeed();
//    speed_send2=chassis_motor[1].getSpeed();
//    speed_send3=chassis_motor[2].getSpeed();
//    speed_send4=chassis_motor[3].getSpeed();
    }
