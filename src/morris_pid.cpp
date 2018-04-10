/*
 * morris_pid.cpp
 *
 *  Created on: Apr 6, 2018
 *      Author: morristseng
 */
#include "morris_pid.h"



int morris_pid::calculate_pid(int current_value, int desire_value){
	int value;
	float derivative;

	if(motor){//motor pid
		error = desire_value - current_value;
		integral += (error*motor_dt);
		derivative = (error-error_prior)/motor_dt;
		value = kp*error + ki*integral + kd*derivative;
		if(value<0){
			value = -value;
		}
		error_prior = error;
		if(value>390){
			value = 390;
		}
	}

	else{//servo pid;

	}

	return value/1.5;

}
