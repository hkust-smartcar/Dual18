/*
 * motorPID.cpp
 *
 *  Created on: 2018¦~1¤ë19¤é
 *      Author: Jake
 */

#include "../inc/PID.h"
#include <cmath>
float PID::getPID(int32_t encoderCount){
	 uint32_t currentTime = System::Time();
	 dTime = currentTime - lastTime;
	 if(dTime ==0){
	  dTime +=1;
	 }
	 if(kI ==0){
		 kI=1;
	 }
	 encoder->Update();
	 currentVelocity = std::abs(encoder->GetCount());
	 currentError = desireVelocity - currentVelocity ;
	 output +=(kP*(currentError - lastError) + kP*dTime * currentError/kI +(kP*kD/dTime)*((currentError - lastError)-(lastError - lastlastError)));
	 lastTime = currentTime;
	 lastlastError = lastError;
	 lastError = currentError;
	 if(output < 0){
	  output = 0;
	 }else if(output >= 200){
		 output = 200;
	 }
	 return output;
}

float PID::getPID(float setPoint, float measuredValue){
	dTime = System::Time() - lastTime;
	if (dTime == 0){
		dTime += 1;
	}
	currentVelocity = measuredValue;
	currentError = setPoint - measuredValue;
	float output = ((currentError) * kP) + ((currentError - lastError) * kD) / (dTime);
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

