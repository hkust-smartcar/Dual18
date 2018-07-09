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
		v[i] = multi[i] * sum[i] / filterCounter;
		sum[i] = 0;
	}
	filterCounter = 0;
	linear = 1.0/v[Mag::magPos::x_left]-1.0/v[Mag::magPos::x_right];
}

void Mag::Calibrate(){
	for (int i = 0; i < 6; i++){
		if (v[i] < min[i]){
			min[i] = v[i];
		}
		if (v[i] > max[i]){
			max[i] = v[i];
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
			b = b && (v[i] < min[i]*1.75*multi[i]);
	}
	return b;
}

float Mag::GetMulti(uint8_t id){
	return multi[id];
}

 void Mag::SetMag(uint8_t id){
	if (id == 1){
		emin = 42;
		emax = 45;
		min[Mag::magPos::x_left] = 7;
		min[Mag::magPos::x_right] = 7;
		max[Mag::magPos::x_left] = 64;
		max[Mag::magPos::x_right] = 63;

		min[Mag::magPos::y_left] = 7;
		min[Mag::magPos::y_right] = 8;
		max[Mag::magPos::y_left] = 66;
		max[Mag::magPos::y_right] = 66;
	}else if (id == 2){
		emin = 45;
		emax = 49;
		min[Mag::magPos::x_left] = 7;
		min[Mag::magPos::x_right] = 7;
		max[Mag::magPos::x_left] = 55;
		max[Mag::magPos::x_right] = 64;

		min[Mag::magPos::y_left] = 8;
		min[Mag::magPos::y_right] = 9;
		max[Mag::magPos::y_left] = 63;
		max[Mag::magPos::y_right] = 71;
	}
	multi[Mag::magPos::x_left] = 70.0/max[Mag::magPos::x_left];
	multi[Mag::magPos::x_right] = 70.0/max[Mag::magPos::x_right];
	multi[Mag::magPos::y_left] = 80.0/max[Mag::magPos::y_left];
	multi[Mag::magPos::y_right] = 80.0/max[Mag::magPos::y_right];
 }

float Mag::GetAllign(){
	return 1.0/(v[0*2]+40)-1.0/(v[0*2+1] + 10);
}

bool Mag::isLoop(){
	return ((v[Mag::magPos::x_left]+v[Mag::magPos::x_right] > (max[Mag::magPos::x_left]+max[Mag::magPos::x_right])*multi[Mag::magPos::x_left]) ||
			(v[Mag::magPos::x_left] > 68 && v[Mag::magPos::x_right] > 68 && v[Mag::magPos::y_left]+v[Mag::magPos::y_right] < 40));
}

bool Mag::unlikelyCrossRoad(){
	return (v[Mag::magPos::y_left] < 18 && v[Mag::magPos::y_right] < 18);
}

bool Mag::outLoop(bool rightLoop){
	float multi = 0.7;
	return ((!rightLoop && v[Mag::magPos::x_left] > max[Mag::magPos::x_left] * multi && v[Mag::magPos::y_left] > max[Mag::magPos::y_left] * multi )||
			(rightLoop && v[Mag::magPos::x_right] > max[Mag::magPos::x_right] * multi && v[Mag::magPos::y_right] > max[Mag::magPos::y_right] * multi));
}

bool Mag::isMidLoop(){
	return (abs(v[Mag::magPos::y_left]-v[Mag::magPos::y_right]) < 10 && v[Mag::magPos::y_left]+v[Mag::magPos::y_right] < 80);
}
