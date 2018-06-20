/*
 * motorPID.h
 *
 *  Created on: 2018¦~1¤ë19¤é
 *      Author: Jake
 */

#ifndef MOTORPID_H_
#define MOTORPID_H_
#include <libsc/system.h>
#include "libsc/dir_encoder.h"

using libsc::DirEncoder;
using libsc::System;
class PID {
public:
	PID(float KP, float KD):kP(KP),kD(KD){};
	PID(float KP, float KI, float KD, DirEncoder* Encoder, bool direction):kP(KP),kI(KI),kD(KD),encoder(Encoder),dir(direction){};
	float getPID();
	float getPID(float setPoint, float measuredValue);
	float getkP(){return kP;}
	void setkP(float v){kP = v;}
	float getkI(){return kI;}
	void setkI(float v){kI = v;}
	float getkD(){return kD;}
	void setkD(float v){kD = v;}
	float getcurrentVelocity(){return currentVelocity;}
	float getDesiredVelocty(){return desireVelocity;}
	void setDesiredVelocity(float v){desireVelocity = v;}
	float getdTerm(){return dTerm;}
	virtual ~PID();
private:
	float kP = 0.0;
	float kI = 0.0;
	float kD = 0.0;
	bool dir = false;
	uint32_t lastTime = 0, dTime = 0;
	float lastVelocity = 0;
	float currentVelocity = 0;
	float desireVelocity = 0;
	float accumulateError = 0;
	float lastError = 0;
	float currentError = 0;
	float output = 0;
	static int counter;
	float dTerm = 0.0;
	DirEncoder * encoder = nullptr;
};

#endif /* MOTORPID_H_ */
