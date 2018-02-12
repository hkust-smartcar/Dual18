/*
 * motorPID.h
 *
 *  Created on: 2018�~1��19��
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
	PID(float KP, float KI, float KD, DirEncoder* Encoder):kP(KP),kI(KI),kD(KD),encoder(Encoder){};
	float getPID(int32_t encoderCount);
	float getkP(){return kP;}
	void setkP(float v){kP = v;}
	float getkI(){return kI;}
	void setkI(float v){kI = v;}
	float getkD(){return kD;}
	void setkD(float v){kD = v;}
	float getcurrentVelocity(){return currentVelocity;}
	float getDesiredVelocty(){return desireVelocity;}
	void setDesiredVelocity(float v){desireVelocity = v;}

	virtual ~PID();
private:
	float kP = 0.0;
	float kI = 0.0;
	float kD = 0.0;
	uint32_t lastTime = 0, dTime = 0;
	float currentVelocity = 0;
	float desireVelocity = 0;
	float accumlateError = 0;
	float lastError = 0;
	float currentError = 0;
	DirEncoder * encoder = nullptr;
};

#endif /* MOTORPID_H_ */