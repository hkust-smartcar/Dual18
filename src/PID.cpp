/*
 * motorPID.cpp
 *
 *  Created on: 2018¦~1¤ë19¤é
 *      Author: Jake
 */

#include "../inc/PID.h"
#include <cmath>
int PID::counter = 0;
float PID::getPID(){
	 uint32_t currentTime = System::Time();
	 dTime = currentTime - lastTime;
	 if(dTime ==0){
	  dTime +=1;
	 }
	 if(kI == 0){
		 kI=1;
	 }
	 encoder->Update();
	 currentVelocity = std::abs(encoder->GetCount()) + 0.0;
	 currentError = desireVelocity - currentVelocity ;
	 output +=(kP*(currentError - lastError) + kP*dTime * currentError/kI +(kP*kD/dTime)*((currentError - lastError)-(lastError - lastlastError)));
	 lastTime = currentTime;
	 lastlastError = lastError;
	 lastError = currentError;
	 if(output < -1000){
	  output = -1000;
	 }else if(output >= 1000){
		 output = 1000;
	 }
	 if(output >= 500 && currentVelocity < desireVelocity){
		 counter++;
	 }
	 if(counter >= 40){
		 output = 0;
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
	if(measuredValue >= -0.50 && measuredValue <= 0.50){
		output = ((currentError) * kP / 2) + dTerm;
	}else{
		output = ((currentError) * kP) + dTerm;
	}
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

