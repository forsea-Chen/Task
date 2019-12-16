/*
 * DR16_Backage.h
 *
 *  Created on: 2019年11月3日
 *      Author: Administrator
 */

#ifndef TOOL_DR16_BACKAGE_H_
#define TOOL_DR16_BACKAGE_H_

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Private  ------------------------------------------------------------------*/
#ifdef __cplusplus
//键位宏定义
#define _W          0
#define _S          1
#define _A          2
#define _D          3
#define _SHIFT      4
#define _CTRL       5
#define _Q          6
#define _E          7
#define _R          8
#define _F          9
#define _G          10
#define _Z          11
#define _X          12
#define _C          13
#define _V          14
#define _B          15
#define _Mouse_L    16
#define _Mouse_R    17


//DR16数据包内容
__packed struct DR16_DataPack_Typedef
{
  uint64_t ch0:11;
  uint64_t ch1:11;
  uint64_t ch2:11;
  uint64_t ch3:11;
  uint64_t s2:2;
  uint64_t s1:2;
  int64_t x:16;
  int64_t y:16;
  int64_t z:16;
  uint64_t press_l:8;
  uint64_t press_r:8;
  uint64_t key:16;
};

//手柄上面两挡位开关状态
enum SW_Status_Typedef
{
    NONE = 0,
    UP = 1,
    MID = 3,
    DOWN = 2,
};

//按键类型定义
struct Key_Typedef
{
    bool Pressed;   //是否按下
    bool Triggered; //是否触发过函数，用来执行点击事件
};


#ifndef __LinkageStatus_DEFINED
#define __LinkageStatus_DEFINED
enum LinkageStatus_Typedef
{
  Connection_Lost = 0U,
  Connection_Established,
};
#endif

/* Exported ------------------------------------------------------------------*/
#define Ignore_Limit 0.05 //线性死区,手柄或鼠标的归一化后的绝对值小于此值时自动视为0

/* DR16类型 */
class DR16_Classdef
{
    private:
        LinkageStatus_Typedef Status;   //连接状态状态
        uint32_t     last_check_time;   //在线检测
        DR16_DataPack_Typedef DataPack; //数据包

        //两个摇杆四个方向与鼠标三个方向速度归一化后的值
        float RX_Norm,RY_Norm,LX_Norm,LY_Norm,MouseX_Norm,MouseY_Norm,MouseZ_Norm;
        Key_Typedef Key[18];  //16个键的相关信息
        float MouseCoefficient;  //鼠标动作乘的系数
        void Key_Process(void); //按键处理
    public:
        DR16_Classdef();
        void DataCapture(DR16_DataPack_Typedef* captureData);  //抓取并更新数据包

        uint64_t GetCh0(void);
        uint64_t GetCh1(void);
        uint64_t GetCh2(void);
        uint64_t GetCh3(void);
        SW_Status_Typedef GetS2(void);
        SW_Status_Typedef GetS1(void);
        int64_t GetMouseX(void);
        int64_t GetMouseY(void);
        int64_t GetMouseZ(void);
        uint64_t GetPress_L(void);
        uint64_t GetPress_R(void);
        uint64_t Getkey(void);

        //归一化后的通道0123、鼠标XYZ值
        float Get_RX_Norm(void);
        float Get_RY_Norm(void);
        float Get_LX_Norm(void);
        float Get_LY_Norm(void);
        float Get_MouseX_Norm(void);
        float Get_MouseY_Norm(void);
        float Get_MouseZ_Norm(void);

        //用于判断某个按键是否按下
        bool IsKeyPress(int _key);

        /*状态相关操作*/
        void Check_Link(uint32_t current_check_time);       //在线检测
        void SetStatus(LinkageStatus_Typedef para_status);  //更新接受信号状态
        LinkageStatus_Typedef GetStatus(void);              //获得接受信号状态

        //按键触发，按一下触发一次功能(提供四个重载版本，分别对应是/否调出返回值与类内/通用函数)
        template<typename Ret_T, typename... Arg_Ts>
        bool Func_Trigger(int _key,Ret_T(*func)(Arg_Ts...),Arg_Ts... args)
        {
            if(Key[_key].Triggered == false && Key[_key].Pressed)
            {
                Key[_key].Triggered = true;
                func(args...);
                return true;
            }
            return false;
        }
        template<typename Ret_T,typename Obj_T, typename... Arg_Ts>
        bool Func_Trigger(int _key,Obj_T& Obj, Ret_T(Obj_T::*func)(Arg_Ts...),Arg_Ts... arg)
        {
            if(Key[_key].Triggered == false && Key[_key].Pressed)
            {

                Key[_key].Triggered = true;
                (Obj.*func)(arg...);
                return true;
            }
            return false;
        }
        template<typename Ret_T, typename... Arg_Ts>
        bool Func_Trigger(int _key,Ret_T& retv_recver,Ret_T(*func)(Arg_Ts...),Arg_Ts... args)
        {
            if(Key[_key].Triggered == false && Key[_key].Pressed)
            {
                Key[_key].Triggered = true;
                retv_recver = func(args...);
                return true;
            }
            return false;
        }
        template<typename Ret_T,typename Obj_T, typename... Arg_Ts>
        bool Func_Trigger(int _key,Ret_T& retv_recver, Obj_T& Obj, Ret_T(Obj_T::*func)(Arg_Ts...),Arg_Ts... arg)
        {
            if(Key[_key].Triggered == false && Key[_key].Pressed)
            {

                Key[_key].Triggered = true;
                retv_recver = (Obj.*func)(arg...);
                return true;
            }
            return false;
        }

        //按键绑定，按住就执行功能(提供四个重载版本，分别对应是/否调出返回值与类内/通用函数)
        template<typename Ret_T, typename... Arg_Ts>
        bool Func_Bind(int _key,Ret_T(*func)(Arg_Ts...),Arg_Ts... args)
        {
            if(Key[_key].Pressed)
            {
                func(args...);
                return true;
            }
            return false;
        }
        template<typename Ret_T,typename Obj_T, typename... Arg_Ts>
        bool Func_Bind(int _key,Obj_T& Obj, Ret_T(Obj_T::*func)(Arg_Ts...),Arg_Ts... arg)
        {
            if(Key[_key].Pressed)
            {
                (Obj.*func)(arg...);
                return true;
            }
            return false;
        }
        template<typename Ret_T, typename... Arg_Ts>
        bool Func_Bind(int _key,Ret_T& retv_recver,Ret_T(*func)(Arg_Ts...),Arg_Ts... args)
        {
            if(Key[_key].Pressed)
            {
                retv_recver = func(args...);
                return true;
            }
            return false;
        }
        template<typename Ret_T,typename Obj_T, typename... Arg_Ts>
        bool Func_Bind(int _key,Ret_T& retv_recver, Obj_T& Obj, Ret_T(Obj_T::*func)(Arg_Ts...),Arg_Ts... arg)
        {
            if(Key[_key].Pressed)
            {
                retv_recver = (Obj.*func)(arg...);
                return true;
            }
            return false;
        }
};

float DeadZone_Process(float num,float DZ_min, float DZ_max, float DZ_num);
#endif




#endif /* TOOL_DR16_BACKAGE_H_ */