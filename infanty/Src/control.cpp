/*
 * control.cpp
 *
 *  Created on: Oct 5, 2019
 *      Author: Administrator
 */

//#include <stm32f4xx_hal_can.h>
//#include <sys/_stdint.h>
#include "control.h"

Motor_C620 chassis_motor[] = { Motor_C620(1), Motor_C620(2), Motor_C620(3), Motor_C620(4) };
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

