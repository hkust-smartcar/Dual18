<<<<<<< HEAD
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

=======
/*
 * motorPID.cpp
 *
 *  Created on: 2018ï¿½~1ï¿½ï¿½19ï¿½ï¿½
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
	 currentVelocity = std::abs(encoder->GetCount()) + 0.0;
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
>>>>>>> morris_branch
