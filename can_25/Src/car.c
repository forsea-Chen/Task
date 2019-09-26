/*
 * car.c
 *
 *  Created on: 2019年9月27日
 *      Author: Administrator
 */
#include "main.h"


void car_run(int speed)
    {
	motor_run(0x201,speed);
	motor_run(0x202,-speed);
	motor_run(0x203,-speed);
	motor_run(0x204,speed);
    }
void car_back(int speed)
    {
	motor_run(0x201,-speed);
	motor_run(0x202,speed);
	motor_run(0x203,speed);
	motor_run(0x204,-speed);
    }
void car_stop()
    {
	motor_run(0x201,0);
	motor_run(0x202,0);
	motor_run(0x203,0);
	motor_run(0x204,0);
    }
void car_leftmove(int speed)
    {
	motor_run(0x201,speed);
	motor_run(0x202,speed);
	motor_run(0x203,-speed);
	motor_run(0x204,-speed);
    }
void car_rightmove(int speed)
    {
	motor_run(0x201,-speed);
	motor_run(0x202,-speed);
	motor_run(0x203,speed);
	motor_run(0x204,speed);
    }
void car_rightturn(int speed)
    {
	motor_run(0x201,-speed);
	motor_run(0x202,-speed);
	motor_run(0x203,speed);
	motor_run(0x204,speed);
    }
