/*
 * car_RM.c
 *
 *  Created on: Sep 28, 2019
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
	motor_run(0x201,speed);
	motor_run(0x202,speed);
	motor_run(0x203,speed);
	motor_run(0x204,speed);
    }
void car_leftturn(int speed)
    {
	motor_run(0x201,-speed);
	motor_run(0x202,-speed);
	motor_run(0x203,-speed);
	motor_run(0x204,-speed);
    }

void move(int vx,int vy,int w)
    {
	int v1,v2,v3,v4;
	v1=vy+vx-w*20;
	v2=vy-vx+w*20;
	v3=vy+vx+w*20;
	v4=vy-vx-w*20;
	motor_moni(v1,v2,v3,v4);
    }

