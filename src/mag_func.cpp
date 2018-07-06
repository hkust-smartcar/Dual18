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
		min[Mag::magPos::x_left] = 8;
		min[Mag::magPos::x_right] = 8;
		max[Mag::magPos::x_left] = 68;
		max[Mag::magPos::x_right] = 73;
		emin = 50;
		emax = 52;

		min[Mag::magPos::y_left] = 7;
		min[Mag::magPos::y_right] = 6;
		max[Mag::magPos::y_left] = 67;
		max[Mag::magPos::y_right] = 57;
	}else if (id == 2){
		min[Mag::magPos::x_left] = 7;
		min[Mag::magPos::x_right] = 7;
		max[Mag::magPos::x_left] = 68;
		max[Mag::magPos::x_right] = 64;
		emin = 40;
		emax = 43;

		min[Mag::magPos::y_left] = 9;
		min[Mag::magPos::y_right] = 8;
		max[Mag::magPos::y_left] = 80;
		max[Mag::magPos::y_right] = 87;
	}
	multi[Mag::magPos::x_left] = 50.0/(emin+emax)*2;
	multi[Mag::magPos::x_right] = 50.0/(emin+emax)*2;
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

bool Mag::outLoop(){
	return ((v[Mag::magPos::x_left] > max[Mag::magPos::x_left] * 0.9 && v[Mag::magPos::y_left] > max[Mag::magPos::y_left]) * 0.9 ||
			(v[Mag::magPos::x_right] > max[Mag::magPos::x_right] * 0.9 && v[Mag::magPos::y_right] > max[Mag::magPos::y_right] * 0.9));
}
