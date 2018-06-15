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
	 if(dTime ==0){
	  dTime +=1;
	 }

	 encoder->Update();

	 if(encoder->GetCount()<500){
		 if(encoder->GetCount()<0)
			 currentVelocity =-1*(encoder->GetCount());
		 else
			 currentVelocity =encoder->GetCount();
	 }
	 currentError = desireVelocity - currentVelocity ;
	 //	 output +=(kP*(currentError - lastError) + kP*dTime * currentError/kI +(kP*kD/dTime)*((currentError - lastError)-(lastError - lastlastError)));
	 output += kP*(currentError) + kD*(currentError - lastError);
	 lastTime = currentTime;
	 lastlastError = lastError;
	 lastError = currentError;
	 if(output < -1000){
	  output = -1000;
	 }else if(output >= 1000){
		 output = 1000;
	 }
	 if(output >= 500 && currentVelocity < (desireVelocity / 2)){
		 counter++;
	 }
	 if(counter >= 80){
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
	if(measuredValue >= -0.35 && measuredValue <= 0.35){
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
