/*
 * DR16_Backage.cpp
 *
 *  Created on: 2019年11月3日
 *      Author: Administrator
 */


/**
  ******************************************************************************
  * @file   ：Remote_DR16.cpp
  * @brief  ：DJI(大疆创新科技)DR16接收机和遥控模块.由于本文件使用了变长参数模板(C++
  *           11新功能)，在使用本文件时应当注意，用keil编译时需在Options for Target
  *           'balabala'(魔术棒)->C/C++->Misc Control处加上--cpp11.
  * @author ：YuyongHu (867115311@qq.com)
  * @date   ：April,2019
  * @version：2.0.1
  ******************************************************************************
  ==============================================================================
                     ##### How to use this module #####
  ==============================================================================
  (1)
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/

#include "DR16_Backage.h"

/* Functions -----------------------------------------------------------------*/
DR16_Classdef::DR16_Classdef()
{
    MouseCoefficient = 128;    //鼠标系数初始化
    Status = Connection_Lost;    //状态初始化
    for(short i=0;i<16;i++)  //键值初始化
    {
        Key[i].Pressed=false;
        Key[i].Triggered=false;
    }
    //摇杆值初始化
    DataPack.ch0 = 1024;
    DataPack.ch1 = 1024;
    DataPack.ch2 = 1024;
    DataPack.ch3 = 1024;
}

/**
 * @brief  获取数据包函数，将DR16接收器收到的数据包的值赋给DR16的成员变量DataPack并进行键盘按键消抖处理
 * @param  captureData:收到的数据包指针
 * @return None.
 */
void DR16_Classdef::DataCapture(DR16_DataPack_Typedef* captureData)
{
    DataPack = *captureData;   //DR16数据包接收
    //各杂七杂八通道值归一化处理
    RX_Norm = DeadZone_Process((float)(DataPack.ch0-1024)/660.0f,-Ignore_Limit,Ignore_Limit,0);
    RY_Norm = DeadZone_Process((float)(DataPack.ch1-1024)/660.0f,-Ignore_Limit,Ignore_Limit,0);
    LX_Norm = DeadZone_Process((float)(DataPack.ch2-1024)/660.0f,-Ignore_Limit,Ignore_Limit,0);
    LY_Norm = DeadZone_Process((float)(DataPack.ch3-1024)/660.0f,-Ignore_Limit,Ignore_Limit,0);

    float temp;

    temp = MouseCoefficient*((float)DataPack.x)/32768.0f;
    temp=temp>1?1:temp;
    temp=temp<-1?-1:temp;
    MouseX_Norm = temp;

    temp = MouseCoefficient*((float)DataPack.y)/32768.0f;
    temp=temp>1?1:temp;
    temp=temp<-1?-1:temp;
    MouseY_Norm = temp;


    temp = MouseCoefficient*((float)DataPack.z)/32768.0f;
    temp=temp>1?1:temp;
    temp=temp<-1?-1:temp;
    MouseZ_Norm = temp;

    Key_Process();      //按键处理
}

/**
 * @brief  按键处理 Key Process
 * @param  None.
 * @return None.
 */
void DR16_Classdef::Key_Process(void)
{
    for(short i=0;i<16;i++)
    {
        //检测到对应按键按下就置key结构数组相关位
        if(DataPack.key & (0x01<<i))
            Key[i].Pressed = true;
        else
        {
            Key[i].Pressed = false;
            Key[i].Triggered = false;
        }
    }
    //鼠标左右键处理
    if(DataPack.press_l == 0x01)
        Key[_Mouse_L].Pressed = true;
    else
    {
        Key[_Mouse_L].Pressed = false;
        Key[_Mouse_L].Triggered = false;
    }
    if(DataPack.press_r == 0x01)
        Key[_Mouse_R].Pressed = true;
    else
    {
        Key[_Mouse_R].Pressed = false;
        Key[_Mouse_R].Triggered = false;
    }
}

/**
 * @brief  下面一串Getxxx函数功能都是获得数据包中的Get后面的数据的值，
           名字比注释好懂就不重复注释了
 * @param  None.
 * @return Get后面的数据的值
 */
uint64_t DR16_Classdef::GetCh0(void)
{
    return DataPack.ch0;
}

uint64_t DR16_Classdef::GetCh1(void)
{
    return DataPack.ch1;
}


uint64_t DR16_Classdef::GetCh2(void)
{
    return DataPack.ch2;
}


uint64_t DR16_Classdef::GetCh3(void)
{
    return DataPack.ch3;
}

SW_Status_Typedef DR16_Classdef::GetS2(void)
{
    return (SW_Status_Typedef)DataPack.s2;
}
SW_Status_Typedef DR16_Classdef::GetS1(void)
{
    return (SW_Status_Typedef)DataPack.s1;
}
int64_t DR16_Classdef::GetMouseX(void)
{
    return DataPack.x;
}
int64_t DR16_Classdef::GetMouseY(void)
{
    return DataPack.y;
}
int64_t DR16_Classdef::GetMouseZ(void)
{
    return DataPack.z;
}
uint64_t DR16_Classdef::GetPress_L(void)
{
    return DataPack.press_l;
}
uint64_t DR16_Classdef::GetPress_R(void)
{
    return DataPack.press_r;
}
uint64_t DR16_Classdef::Getkey(void)
{
    return DataPack.key;
}
/**
 * @brief  归一化后的通道0123、鼠标XYZ值(Left_X_Axis,Right_Y_Axis,balabala)
 * @param  None.
 * @retval -1~1之间的通道值
 */
float DR16_Classdef::Get_RX_Norm(void)
{
    return RX_Norm;
}
float DR16_Classdef::Get_RY_Norm(void)
{
    return RY_Norm;
}
float DR16_Classdef::Get_LX_Norm(void)
{
    return LX_Norm;
}
float DR16_Classdef::Get_LY_Norm(void)
{
    return LY_Norm;
}
float DR16_Classdef::Get_MouseX_Norm(void)
{
    return MouseX_Norm;
}
float DR16_Classdef::Get_MouseY_Norm(void)
{
    return MouseY_Norm;
}
float DR16_Classdef::Get_MouseZ_Norm(void)
{
    return MouseZ_Norm;
}

/**
 * @brief  用于判断某个按键是否按下,组合键立马判断
 * @param _key 头文件中宏定义的key键，如_w,_s等
 * @retval 按下为ture，没按下为false
 */
bool DR16_Classdef::IsKeyPress(int _key)
{
    return Key[_key].Pressed;
}

/**
 * @brief  设置DR16成员变量status的值，用于设置DR16离线还是在线
 * @param  para_status 要设置的状态
 * @@return None.
 */
void DR16_Classdef::SetStatus(LinkageStatus_Typedef para_status)
{
  last_check_time = 0;
  Status = para_status;
}

/**
 * @brief   得到DR16成员变量status的值，常用于判断DR16是否在线
 * @param   None.
 * @@return Offline DR16离线   Online DR16在线
 */
LinkageStatus_Typedef DR16_Classdef::GetStatus(void)
{
  return Status;
}


/**
 * @brief  死区处理，常用于消除零点附近的微小误差
 * @param  num:要处理的数; DZ_min,DZ_max:死区范围;DZ_num:落在死区内时返回的值
 * @@return 处理后的结果
 */
float DeadZone_Process(float num,float DZ_min, float DZ_max, float DZ_num)
{
    //若在死区内则返回死区值
    if(num<DZ_max&&num>DZ_min)
    {
        return DZ_num;
    }
    else
        return num;
}

/**
* @brief  连接确认，更新设备的连接状态。
          Is_Connected()：返回当前检测到的设备状态
          Check_Link()  ：通过检测‘connection_status’是否刷新
          确定连接状态
* @param  current_time(ms).
* @return 0:offline.
          1:online
* @author
*/
void DR16_Classdef::Check_Link(uint32_t current_check_time)
{
  if(last_check_time == 0)
  {
    last_check_time = current_check_time;
  }
  else
  {
    if(current_check_time - last_check_time > 50)
    {
      //计数溢出
      if(current_check_time - last_check_time > 1000) __nop();
      else Status = Connection_Lost;

      last_check_time = 0;
    }
    else __nop();
  }
}

/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/


