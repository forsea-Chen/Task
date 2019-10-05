/*
 * Car_1.01.c
 *
 *  Created on: Oct 4, 2019
 *      Author: Administrator
 */
#include "main.h"

void move(int16_t vx,int16_t vy,int16_t w)
    {
	int v1,v2,v3,v4;
	v1=vy+vx-w*20;
	v2=-(vy-vx+w*20);
	v3=-(vy+vx+w*20);
	v4=vy-vx-w*20;
	motor_moni(v1,v2,v3,v4);
    }
