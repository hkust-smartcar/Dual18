/*
 * mag_func.cpp
 *
 *  Created on: 21 Jun 2018
 *      Author: PWS
 */

#include "../inc/mag_func.h"

void Mag::TakeSample(){
	filterCounter++;
	sum[0] += mag0.GetResult();
	sum[1] += mag1.GetResult();
	sum[2] += mag2.GetResult();
	sum[3] += mag3.GetResult();
	sum[4] += mag4.GetResult();
	sum[5] += mag5.GetResult();
}

void Mag::Update(){
	for (int i = 0; i < 6; i++){
		raw[i] = sum[i] / filterCounter;
		if (raw[i] > 0){
			v[i] = multi[i] * (raw[i] - min[i]);
		}
		sum[i] = 0;
	}
	filterCounter = 0;
	xLinear = 1.0/v[Mag::magPos::x_left]-1.0/v[Mag::magPos::x_right];
	yLinear = v[Mag::magPos::y_right]-v[Mag::magPos::y_left];
}

void Mag::Calibrate(){
	for (int i = 0; i < 6; i++){
		if (raw[i] < min[i]){
			min[i] = raw[i];
		}
		if (raw[i] > max[i]){
			max[i] = raw[i];
		}
	}
	if (v[Mag::magPos::x_left] == v[Mag::magPos::x_right]){
		if (v[Mag::magPos::x_left] < emin){
			emin = v[Mag::magPos::x_left];
		}
		if (v[Mag::magPos::x_left] > emax){
			emax = v[Mag::magPos::x_left];
		}
	}
}

bool Mag::noMagField(){
	bool b = true;
	for (int i = 0; i < 6; i++){
		if (i != 2 && i != 3)
			b = b && (v[i] < 10);
	}
	return b;
}

 void Mag::InitMag(uint8_t car_id){
	if (car_id == 1){
		emin = 52;
		emax = 55;
		min[Mag::magPos::x_left] = 7;
		min[Mag::magPos::x_right] = 7;
		max[Mag::magPos::x_left] = 62;
		max[Mag::magPos::x_right] = 66;

		min[Mag::magPos::y_left] = 7;
		min[Mag::magPos::y_right] = 8;
		max[Mag::magPos::y_left] = 72;
		max[Mag::magPos::y_right] = 72;
	}else if (car_id == 2){
		emin = 48;
		emax = 50;
		min[Mag::magPos::x_left] = 10;
		min[Mag::magPos::x_right] = 7;
		max[Mag::magPos::x_left] = 56;
		max[Mag::magPos::x_right] = 58;

		min[Mag::magPos::y_left] = 8;
		min[Mag::magPos::y_right] = 7;
		max[Mag::magPos::y_left] = 62;
		max[Mag::magPos::y_right] = 71;
//initial value for calibration
//		emin = 200;
//		emax = 4;
//		min[Mag::magPos::x_left] = 70;
//		min[Mag::magPos::x_right] = 70;
//		max[Mag::magPos::x_left] = 6;
//		max[Mag::magPos::x_right] = 6;
//
//		min[Mag::magPos::y_left] = 70;
//		min[Mag::magPos::y_right] = 80;
//		max[Mag::magPos::y_left] = 6;
//		max[Mag::magPos::y_right] = 6;
	}
	multi[Mag::magPos::x_left] = 80.0/(max[Mag::magPos::x_left]-min[Mag::magPos::x_left]);
	multi[Mag::magPos::x_right] = 80.0/(max[Mag::magPos::y_right]-min[Mag::magPos::y_right]);
	multi[Mag::magPos::y_left] = 80.0/(max[Mag::magPos::y_left]-min[Mag::magPos::y_left]);
	multi[Mag::magPos::y_right] = 80.0/(max[Mag::magPos::y_right]-min[Mag::magPos::y_right]);
 }

bool Mag::isLoop(){
	return (v[Mag::magPos::x_left] > 60 && v[Mag::magPos::x_right] > 60);
}

bool Mag::isTwoLine(){
	return (Mag::GetXSum()+Mag::GetYSum() > 160);
}

//need to change
bool Mag::isRightLoop(){
	return (v[Mag::magPos::y_right] + v[Mag::magPos::x_right] > v[Mag::magPos::y_left] + v[Mag::magPos::x_left]);
}

//need to change
bool Mag::unlikelyCrossRoad(){
	return (v[Mag::magPos::y_left] < 15 && v[Mag::magPos::y_right] < 15);
}

//need to change
bool Mag::outLoop(){
	return v[Mag::magPos::x_left] + v[Mag::magPos::x_right] + v[Mag::magPos::y_left] + v[Mag::magPos::y_right] > 200;
}

//need to change
bool Mag::isMidLoop(){
	return (v[Mag::magPos::y_left] < 30 || v[Mag::magPos::y_right] < 30);
}
