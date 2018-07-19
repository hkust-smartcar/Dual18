/*
 * mag_func.cpp
 *
 *  Created on: 21 Jun 2018
 *      Author: PWS
 */

#include "mag_func.h"
#include "func.h"

void Mag::TakeSample(){
	for (int i = 0; i < 20; i++){
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
	return (b && (Mag::GetXSum()+Mag::GetYSum() < 40));
}

 void Mag::InitMag(uint8_t car_id){
	if (car_id == 1){
		min[Mag::magPos::x_left] = 7;
		min[Mag::magPos::x_right] = 5;
		max[Mag::magPos::x_left] = 78;
		max[Mag::magPos::x_right] = 78;

		min[Mag::magPos::y_left] = 11;
		min[Mag::magPos::y_right] = 7;
		max[Mag::magPos::y_left] = 87;
		max[Mag::magPos::y_right] = 81;

//		min[Mag::magPos::x_left] = 255;
//		min[Mag::magPos::x_right] = 255;
//		max[Mag::magPos::x_left] = 0;
//		max[Mag::magPos::x_right] = 0;
//
//		min[Mag::magPos::y_left] = 255;
//		min[Mag::magPos::y_right] = 255;
//		max[Mag::magPos::y_left] = 0;
//		max[Mag::magPos::y_right] = 0;

	}else if (car_id == 2){
		min[Mag::magPos::x_left] = 9;
		min[Mag::magPos::x_right] = 8;
		max[Mag::magPos::x_left] = 65;
		max[Mag::magPos::x_right] = 67;
//8 9
		min[Mag::magPos::y_left] = 9;
		min[Mag::magPos::y_right] = 10;
		max[Mag::magPos::y_left] = 70;
		max[Mag::magPos::y_right] = 71;
//initial value for calibration
//		min[Mag::magPos::x_left] = 255;
//		min[Mag::magPos::x_right] = 255;
//		max[Mag::magPos::x_left] = 0;
//		max[Mag::magPos::x_right] = 0;
//
//		min[Mag::magPos::y_left] = 255;
//		min[Mag::magPos::y_right] = 255;
//		max[Mag::magPos::y_left] = 0;
//		max[Mag::magPos::y_right] = 0;
	}
	multi[Mag::magPos::x_left] = 80.0/(max[Mag::magPos::x_left]-min[Mag::magPos::x_left]);
	multi[Mag::magPos::x_right] = 80.0/(max[Mag::magPos::x_right]-min[Mag::magPos::x_right]);
	multi[Mag::magPos::y_left] = 80.0/(max[Mag::magPos::y_left]-min[Mag::magPos::y_left]);
	multi[Mag::magPos::y_right] = 80.0/(max[Mag::magPos::y_right]-min[Mag::magPos::y_right]);
 }

bool Mag::isLoop(){
	return ((v[Mag::magPos::x_left] > 65 && v[Mag::magPos::x_right] > 65) || Mag::GetXSum() > 160);
}

bool Mag::isTwoLine(){
	return (Mag::GetXSum()+Mag::GetYSum() > 180);
}

bool Mag::unlikelyCrossRoad(){
	return (v[Mag::magPos::y_left] < 35 && v[Mag::magPos::y_right] < 35);
}

bool Mag::outLoop(){
	return (Mag::GetXSum()+Mag::GetYSum() > 250);
}

void Mag::CheckState(uint32_t lastTime, uint32_t &approachTime, carState &magState, float &speed, bool &approaching, bool &isFirst, bool &firstArrived, bool &secondArrived){
	if (magState == kNormal && approaching) {
		magState = kLeave;
		leaveCount = 0;
	} else if (magState == kLeave && v[Mag::magPos::x_left] < 15 && v[Mag::magPos::x_right] < 50){
		if (isFirst){
			magState = kStop;
			speed = 0;
		} else {
			magState = kSide;
			secondArrived = true;
		}
	} else if (magState == kStop && secondArrived){
		magState = kSide;
		speed = aSpeed;
		approachTime = lastTime;
	} else if (magState == kAlign && abs(Mag::GetYDiff()) < 4){
		magState = kSide;
		approachTime = lastTime;
	} else if (magState == kSide && !approaching) {
		magState = kNormal;
		if (isFirst || firstArrived || secondArrived){
			isFirst = false;
			firstArrived = false;
			secondArrived = false;
		}
		speed = hSpeed;
	}
}

float Mag::GetAngle(PID &x_servo, PID &y_servo, PID &align_servo, float &angleX, float &angleY, carState magState, bool left_loop, bool in_loop, bool isCrossRoad, float yTarget){
	float servoAngle = 0;
	if (magState == kNormal || magState == kLoop || magState == kExitLoop){
		float target = 0.0;
		if (magState == kLoop){
			if (left_loop){
				target = 0.015;
			} else{
				target = -0.015;
			}
		} else if (magState == kExitLoop){
			if (left_loop){
				target = 0.011;
			} else{
				target = -0.011;
			}
		}
		angleX = x_servo.getPID(target, xLinear);
		angleY = y_servo.getPID(yTarget, yLinear);
		if (isCrossRoad && magState == kNormal){
			servoAngle = 0.25*angleX + 0.25*angleY;
		} else{
			servoAngle = 0.5*angleX + 0.5*angleY;
		}
	} else if (magState == kEnter){
		angleY = y_servo.getPID(yTarget, yLinear);
		servoAngle = angleY;
	} else if (magState == kOutLoop){
		float target = 0.0;
		if (left_loop){
			target = -0.02;
		} else{
			target = 0.02;
		}
		angleX = x_servo.getPID(0, xLinear);
		angleY = y_servo.getPID(yTarget, yLinear);
		servoAngle = 0.5*angleX + 0.5*angleY;
	} else if (magState == kLeave){
		servoAngle = Max(300-(leaveCount*20), 100);
		leaveCount++;
	} else if (magState == kStop){
		servoAngle = -400;
	} else if (magState == kAlign){
		servoAngle = -align_servo.getPID(40, v[Mag::magPos::x_right]);
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
	}
	return servoAngle;
 }
