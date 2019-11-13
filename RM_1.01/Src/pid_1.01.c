/*
 * pid_1.01.c
 *
 *  Created on: Oct 4, 2019
 *      Author: Administrator
 */


#include "main.h"
int16_t out;
int16_t error_i=0,error_d=0,error_last=0,error=0;
float kp=10,ki=0.3,kd=0.2;
int16_t PID_OUTPUT(int16_t speed,int16_t target)
    {
	float I_Term,P_Term,D_Term;
	error=target-speed;
	P_Term = kp*error;
	if(speed>=16000)
	    {
		if(error<0)
		    error_i+=error;
	    }
	if(speed<=-16000)
	    {
		if(error>0)
		    error_i+=error;
	    }
	if(error>=8000||error<=-8000)
	    {
		if(error_i>=16000)error_i=16000;
		if(error_i<=-16000)error_i=-16000;
		I_Term=ki*error_i;
	    }
	error_d=error-error_last;
	D_Term=kd*error_d;
	error_last=error;
	out=P_Term+I_Term+D_Term;
	if((error>=0&&error<=10)||(error<0&&error>=-10))
	    {out=0;}
	if(out>=15000)out=15000;
	if(out<=-15000)out=-15000;
	return out;
    }
