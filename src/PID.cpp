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
	float output = ((currentError) * kP) + (accumlateError) * kI * (dTime) + ((currentError - lastError) * kD) / (dTime);
	lastTime = System::Time();
	accumlateError += currentError;
	lastError = currentError;
	return output;
}

PID::~PID() {
	// TODO Auto-generated destructor stub
}

