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
	 dTime = System::Time() - lastTime;
	 lastTime = System::Time();
	 if(dTime == 0){
	  dTime =1;
	 }
	 encoder->Update();
	 if (dir){
		 currentVelocity = -(encoder->GetCount()) / (dTime + 0.0);
	 }else{
		 currentVelocity = encoder->GetCount()/ (dTime + 0.0);
	 }
	 if (currentVelocity > 1000 || currentVelocity < -1000){
		 currentVelocity = lastVelocity;
	 }
	 currentError = desireVelocity - currentVelocity ;
	 accumulateError += currentError;
	 if (accumulateError > 150){
		 accumulateError = 150;
	 }else if (accumulateError < -150){
		 accumulateError = -150;
	 }
	 pTerm = kP * currentError;
	 iTerm = kI * accumulateError;
	 dTerm = kD * (currentError - lastError);
	 output = pTerm + iTerm + dTerm;
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
	pTerm = currentError * kP;
	dTerm = ((currentError - lastError) * kD) / (dTime + 0.0);
	output = pTerm + dTerm;
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
