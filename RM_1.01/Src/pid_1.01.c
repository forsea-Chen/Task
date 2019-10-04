/*
 * pid_1.01.c
 *
 *  Created on: Oct 4, 2019
 *      Author: Administrator
 */


#include "main.h"
int16_t out;
int16_t error_i=0,error_d=0,error_last=0,error=0;
float kp=6,ki=0.23,kd=0.01;
int16_t PID_OUTPUT(int16_t speed,int16_t target)
    {

	error=target-speed;
	error_i+=error;
	if(error_i>=5000)error_i=5000;
	if(error_i<=-5000)error_i=-5000;
	error_d=error-error_last;
	error_last=error;
	out=kp*error+ki*error_i+kd*error_d;
	if((error>=0&&error<=20)||(error<0&&error>=-20))
	    {out=0;}
	if(out>=10000)out=10000;
	if(out<=-10000)out=-10000;
	return out;
    }
