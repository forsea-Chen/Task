/*
 * pid_RM.c
 *
 *  Created on: Sep 28, 2019
 *      Author: Administrator
 */
#include "main.h"
double out;
static double error_i=0,error_d=0,error_last=0,error=0;
static float kp=6,ki=0.23,kd=0.01;
double PID_OUTPUT(int16_t speed,int16_t target)
    {

	error=target-speed;
	error_i+=error;
	if(error_i>=3000)error_i=3000;
	error_d=error-error_last;
	error_last=error;
	out=kp*error+ki*error_i+kd*error_d;
	if((error>=0&&error<=20)||(error<0&&error>=-20))
	    {out=0;}
	return out;
    }

