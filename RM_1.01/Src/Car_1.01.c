/*
 * Car_1.01.c
 *
 *  Created on: Oct 4, 2019
 *      Author: Administrator
 */
#include "main.h"
#include "math.h"

void move(int16_t vx,int16_t vy,float w)
    {
	int v1,v2,v3,v4,x,y;
	wx=3000;
	x=vx;
	y=vy;
	vx=y*sin(w)+x*cos(w);//y*(-sin(w))+x*cos(w);//
	vy=y*cos(w)+x*(-sin(w));//y*cos(w)+x*sin(w);//
	v1=vy+vx-wx;
	v2=-(vy-vx+wx);
	v3=-(vy+vx+wx);
	v4=vy-vx-wx
		;
	motor_moni(v1,v2,v3,v4);
    }
