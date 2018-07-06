/*
 * motorPID.cpp
 *
 *  Created on: 2018�~1��19��
 *      Author: Jake
 */

#include "../inc/PID.h"
#include <cmath>
int PID::counter = 0;
float PID::getPID(){
	 uint32_t currentTime = System::Time();
	 dTime = currentTime - lastTime;
	 if(dTime == 0){
	  dTime =1;
	 }
	 encoder->Update();
	 if (dir){
		 currentVelocity = -(encoder->GetCount());
	 }else{
		 currentVelocity = encoder->GetCount();
	 }
	 if (currentVelocity > 1000 || currentVelocity < -1000){
		 currentVelocity = lastVelocity;
	 }
	 currentError = desireVelocity - currentVelocity ;
	 accumulateError += currentError;
	 if (accumulateError > 300){
		 accumulateError = 300;
	 }else if (accumulateError < -300){
		 accumulateError = -300;
	 }
	 output = kP*currentError + kI*accumulateError + kD*(currentError - lastError);
	 lastTime = currentTime;
	 lastError = currentError;
	 lastVelocity = currentVelocity;
	 if(output < -8.5){
	  output = -8.5;
	 }else if(output >= 8.5){
		 output = 8.5;
	 }
	 return output;
}

float PID::getPID(float setPoint, float measuredValue){
	dTime = System::Time() - lastTime;
	lastTime = System::Time();
	if (dTime == 0){
		dTime += 1;
	}
	currentVelocity = measuredValue;
	currentError = setPoint - measuredValue;
	float output = 0;
	dTerm = ((currentError - lastError) * kD) / (dTime);
	output = ((currentError) * kP) + dTerm;
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
