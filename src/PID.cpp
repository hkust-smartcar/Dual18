/*
 * motorPID.cpp
 *
 *  Created on: 2018¦~1¤ë19¤é
 *      Author: Jake
 */

#include "../inc/PID.h"

float PID::getPID(int32_t encoderCount){
	this->desireVelocity = desireVelocity;
	dTime = System::Time() - lastTime;
	encoder->Update();
	currentVelocity = encoderCount;
	currentError = desireVelocity + encoderCount;
	accumlateError += currentError;
	float output = ((currentError) * kP) + ((accumlateError) * kI * (dTime + 1)) + (((currentError - lastError) * kD) / (dTime + 1)); //prevent division by 0 error
	lastTime = System::Time();
	lastError = currentError;
	if(output >= 200){
		output = 200;
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

