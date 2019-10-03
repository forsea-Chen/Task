/*
 * pid_RM.c
 *
 *  Created on: Sep 28, 2019
 *      Author: Administrator
 */
#include "main.h"
int16_t out;
static int16_t error_i=0,error_d=0,error_last=0,error=0;
static float kp=1.3,ki=0.03,kd=0.03;
int16_t PID_OUTPUT(int16_t speed,int16_t target)
    {

	error=target-speed;
	error_i+=error;
	if(error_i>=3000)error_i=3000;
	if(error_i<=-3000)error_i=-3000;
	error_d=error-error_last;
	error_last=error;
	out=kp*error+ki*error_i+kd*error_d;
	if((error>=0&&error<=20)||(error<0&&error>=-20))
	    {out=0;}
	if(out>=10000)out=10000;
	if(out<=-10000)out=-10000;
	return out;
    }

