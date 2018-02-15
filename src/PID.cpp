/*
 * motorPID.cpp
 *
 *  Created on: 2018¦~1¤ë19¤é
 *      Author: Jake
 */

#include "../inc/PID.h"

float PID::getPID(){
	encoder->Update();
	currentVelocity = encoder->GetCount();
	dTime = System::Time() - lastTime;
	currentError = desireVelocity + currentVelocity;

	float Pout = kP * currentError;

	accumlateError += currentError * (dTime + 0.0);
	float Iout = kI * accumlateError;

	float derivative = (currentError - lastError) / (dTime + 0.0);
	float Dout = kD * derivative;


	float output = Pout + Iout + Dout;

	lastTime = System::Time();
	lastError = currentError;
	if(output >= 250){
		output = 250;
	}
	else if(output <= 0){
		output = 0;
	}
	return output;
}

float PID::getPID(float setPoint, float measuredValue){
	dTime = System::Time() - lastTime;
	currentVelocity = measuredValue;
	currentError = setPoint - measuredValue;
	float output = ((currentError) * kP) + ((currentError - lastError) * kD) / (dTime + 1);
	lastTime = System::Time();
	lastError = currentError;
	if(output >= 1800){
		output = 1800;
	}else if(output <= 0){
		output = 0;
	}
	return output;
}

PID::~PID() {
	// TODO Auto-generated destructor stub
}

