/*
 * mag_func.cpp
 *
 *  Created on: 21 Jun 2018
 *      Author: PWS
 */

#include "mag_func.h"
#include "func.h"

void Mag::TakeSample(){
	for (int i = 0; i < 3; i++){
		filterCounter++;
		sum[0] += mag0.GetResult();
		sum[1] += mag1.GetResult();
		sum[2] += mag2.GetResult();
		sum[3] += mag3.GetResult();
		sum[4] += mag4.GetResult();
	}
}

void Mag::Update(){
	for (int i = 0; i < 5; i++){
		raw[i] = sum[i] / filterCounter;
		if (raw[i] > min[i]){
			v[i] = multi[i] * (raw[i] - min[i]);
		}
		sum[i] = 0;
	}
	filterCounter = 0;
	xLinear = 1.0/v[Mag::magPos::x_left]-1.0/v[Mag::magPos::x_right];
	yLinear = v[Mag::magPos::y_right]-v[Mag::magPos::y_left];
}

void Mag::Calibrate(){
	for (int i = 0; i < 5; i++){
		if (raw[i] < min[i]){
			min[i] = raw[i];
		}
		if (raw[i] > max[i]){
			max[i] = raw[i];
		}
	}
	multi[Mag::magPos::x_left] = 80.0/(max[Mag::magPos::x_left]-min[Mag::magPos::x_left]);
	multi[Mag::magPos::x_right] = 80.0/(max[Mag::magPos::y_right]-min[Mag::magPos::y_right]);
	multi[Mag::magPos::y_left] = 80.0/(max[Mag::magPos::y_left]-min[Mag::magPos::y_left]);
	multi[Mag::magPos::y_right] = 80.0/(max[Mag::magPos::y_right]-min[Mag::magPos::y_right]);
}

bool Mag::noMagField(){
	bool b = true;
	for (int i = 0; i < 5; i++){
		if (i != 4)
			b = b && (v[i] < 20);
	}
	return (b && (Mag::GetXSum()+Mag::GetYSum() < 35));
}

 void Mag::InitMag(uint8_t car_id, FlashWrapper* flash){

	flash->link_uint8_t(0, &min[Mag::magPos::x_left]);
	flash->link_uint8_t(1, &min[Mag::magPos::x_right]);
	flash->link_uint8_t(2, &max[Mag::magPos::x_left]);
	flash->link_uint8_t(3, &max[Mag::magPos::x_right]);
	flash->link_uint8_t(4, &min[Mag::magPos::y_left]);
	flash->link_uint8_t(5, &min[Mag::magPos::y_right]);
	flash->link_uint8_t(6, &max[Mag::magPos::y_left]);
	flash->link_uint8_t(7, &max[Mag::magPos::y_right]);

//	flash->readFlash();

	if (car_id == 1){
		min[Mag::magPos::x_left] = 9;
		min[Mag::magPos::x_right] = 8;
		max[Mag::magPos::x_left] = 71;
		max[Mag::magPos::x_right] = 71;

		min[Mag::magPos::y_left] = 11;
		min[Mag::magPos::y_right] = 9;
		max[Mag::magPos::y_left] = 69;
		max[Mag::magPos::y_right] = 68;

	}else if (car_id == 2){
		min[Mag::magPos::x_left] = 9;
		min[Mag::magPos::x_right] = 8;
		max[Mag::magPos::x_left] = 68;
		max[Mag::magPos::x_right] = 73;

		min[Mag::magPos::y_left] = 9;
		min[Mag::magPos::y_right] = 10;
		max[Mag::magPos::y_left] = 70;
		max[Mag::magPos::y_right] = 71;
	}

	multi[Mag::magPos::x_left] = 80.0/(max[Mag::magPos::x_left]-min[Mag::magPos::x_left]);
	multi[Mag::magPos::x_right] = 80.0/(max[Mag::magPos::x_right]-min[Mag::magPos::x_right]);
	multi[Mag::magPos::y_left] = 80.0/(max[Mag::magPos::y_left]-min[Mag::magPos::y_left]);
	multi[Mag::magPos::y_right] = 80.0/(max[Mag::magPos::y_right]-min[Mag::magPos::y_right]);
 }

void Mag::ResetMag(){
	min[Mag::magPos::x_left] = 255;
	min[Mag::magPos::x_right] = 255;
	max[Mag::magPos::x_left] = 0;
	max[Mag::magPos::x_right] = 0;

	min[Mag::magPos::y_left] = 255;
	min[Mag::magPos::y_right] = 255;
	max[Mag::magPos::y_left] = 0;
	max[Mag::magPos::y_right] = 0;
}

bool Mag::isLoop(){
	return ((v[Mag::magPos::x_left] > 68 && v[Mag::magPos::x_right] > 68 && Mag::GetYSum() > 25) || Mag::GetXSum() > 160);
}

bool Mag::isTwoLine(){
	return (Mag::GetXSum()+Mag::GetYSum() > 180);
}

bool Mag::unlikelyCrossRoad(){
	return (v[Mag::magPos::y_left] < 35 && v[Mag::magPos::y_right] < 35);
}

bool Mag::outLoop(){
	return (Mag::GetXSum()+Mag::GetYSum() > 240);
}

void Mag::CheckState(uint32_t lastTime, uint32_t &approachTime, carState &magState, float &speed, bool &approaching, bool &isFirst, bool &firstArrived, bool &secondArrived, bool &anotherGG, bool &USsent){
	static uint32_t waitTime = 0;
	if (magState == kNormal && approaching) {
		magState = kLeave;
		leaveCount = 0;
	} else if (magState == kLeave && v[Mag::magPos::x_left] < 25){// && v[Mag::magPos::x_right] < 50
		if (isFirst){
			magState = kStop;
			speed = 0;
			waitTime = lastTime;
		} else {
			magState = kSide;
		}
	} else if (magState == kStop && (secondArrived || !approaching)){
		magState = kSide;
		speed = aSpeed;
		approachTime = lastTime;
	} else if (magState == kStop && lastTime - waitTime > 10000){
		magState = kNormal;
		speed = hSpeed;
	} else if (magState == kSide && (!approaching)) {
		approaching = false;
		magState = kBack;
		if (isFirst || firstArrived || secondArrived){
			isFirst = false;
			firstArrived = false;
			secondArrived = false;
		}
	} else if (magState == kBack && v[Mag::magPos::x_left] > 30){
		magState = kNormal;
		speed = hSpeed;
		USsent = true;
	}
}

float Mag::GetAngle(PID &x_servo, PID &y_servo, PID &align_servo, float &angleX, float &angleY, carState magState, bool left_loop, bool in_loop, float yTarget){
	float servoAngle = 0;
	if (magState == kNormal || magState == kLoop || magState == kExitLoop){
		leaveCount = 0;
		float target = 0.0;
		if (magState == kLoop){
			if (left_loop){
				target = 0.005;
			} else{
				target = -0.005;
			}
		} else if (magState == kExitLoop){
			if (left_loop){
				target = 0.016;
			} else{
				target = -0.016;
			}
		}
		if (v[Mag::magPos::x_left] < 13 || v[Mag::magPos::x_right] < 13){
			angleX = lastX;
		} else {
			angleX = x_servo.getPID(target, xLinear);
			lastX = angleX;
		}
		angleY = y_servo.getPID(yTarget, yLinear);
		if (Mag::isTwoLine() && magState == kNormal){
			servoAngle = 0.5*angleX;
		} else if (v[Mag::magPos::y_left] < 10 && v[Mag::magPos::y_right] < 10){
			servoAngle = 0.5*angleX;
		} else{
			servoAngle = 0.5*angleX + 0.5*angleY;
		}
	} else if (magState == kEnter){
		angleY = y_servo.getPID(yTarget, yLinear);
		servoAngle = angleY;
	} else if (magState == kOutLoop){
		float target = 0.0;
		if (left_loop){
			target = -0.004;
		} else{
			target = 0.004;
		}
		angleX = x_servo.getPID(0, xLinear);
		angleY = y_servo.getPID(yTarget, yLinear);
		servoAngle = 0.5*angleX + 0.25*angleY;
	} else if (magState == kLeave){
		servoAngle = Max(300-(leaveCount*15), 100);
		leaveCount++;
	} else if (magState == kStop){
		servoAngle = -400;
	} else if (magState == kSide){
		if (Mag::GetYSum() > 15){
			servoAngle = 100 - 5 * align_servo.getPID(0, v[Mag::magPos::x_left]);
			angleY = servoAngle;
		} else{
			servoAngle = -align_servo.getPID(40, v[Mag::magPos::x_right]);
			if (v[Mag::magPos::x_right] < 25){
				servoAngle *= 1 + ((25-v[Mag::magPos::x_right]) * 0.005);
			}
			angleX = servoAngle;
		}
//		servoAngle = -align_servo.getPID(40, v[Mag::magPos::x_right]);
	} else if (magState == kBack){
		servoAngle = -Max(300-(leaveCount*15), 100);
	}
	return servoAngle;
 }
