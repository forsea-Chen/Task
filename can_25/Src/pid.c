/*
 * pid.c
 *
 *  Created on: Sep 26, 2019
 *      Author: Administrator
 */
#include "main.h"
double out;
static double error_i=0,error_d=0,error_last=0,error=0;
static float kp=1.2,ki=0.03,kd=0.03;
double PID_OUTPUT(int16_t speed,int16_t target)
    {

	error=target-speed;
	error_i+=error;
	error_d=error-error_last;
	error_last=error;
	out=kp*error+ki*error_i+kd*error_d;
	return out;
    }
