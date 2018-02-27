/*
 * motorPID.cpp
 *
 *  Created on: 2018¦~1¤ë19¤é
 *      Author: Jake
 */

#include "../inc/PID.h"

float PID::getPID(int32_t encoderCount){
	this->desireVelocity = desireVelocity;
	uint32_t nowTime = System::Time();
	dTime = nowTime - lastTime;
	if (dTime == 0){
		dTime += 1;
	}
	encoder->Update();
	currentVelocity = (0 - encoder->GetCount() / dTime);
	currentError = (desireVelocity - currentVelocity);
	accumlateError += encoderCount;
	float output = ((currentError) * kP) + ((accumlateError) * kI * (dTime)) + (((currentError - lastError) * kD) / (dTime));
	lastError = currentError;
	lastTime = nowTime;
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
	if (dTime == 0){
		dTime += 1;
	}
	currentVelocity = measuredValue;
	currentError = (setPoint - measuredValue);
	float output = ((currentError * currentError)/ 2 * kP) + ((currentError - lastError) * kD) / (dTime);
	lastTime = System::Time();
	lastError = currentError;
	if(output >= 900){
		output = 900;
	}else if(output <= -900){
		output = -900;
	}
	return output;
}

PID::~PID() {
	// TODO Auto-generated destructor stub
}

