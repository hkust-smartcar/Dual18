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
		v[i] = multi[i/2] * sum[i] / filterCounter;
		sum[i] = 0;
	}
	filterCounter = 0;
	for (int j = 0; j < 3; j++){
		linear[j] = 1.0/v[j*2]-1.0/v[j*2+1];
	}
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
	for (int i = 0; i < 3; i++){
		if (v[i*2] == v[i*2+1]){
			if (v[i*2] < emin[i]){
				emin[i] = v[i*2];
			}
			if (v[i*2] > emax[i]){
				emax[i] = v[i*2];
			}
		}
	}
}

bool Mag::noMagField(){
	bool b = true;
	for (int i = 0; i < 2; i++){
		b = b && (v[i] < min[i]*1.5*multi[i/2]);
	}
	return b;
}

float Mag::GetLinear(uint8_t pair_id){
	return linear[pair_id];
}

float Mag::GetMulti(uint8_t pair_id){
	return multi[pair_id];
}

 void Mag::SetMag(uint8_t id){
	if (id == 1){
		min[0] = 6;
		min[1] = 8;
		max[0] = 70;
		max[1] = 68;
		emin[0] = 46;
		emax[0] = 50;
	}else if (id == 2){
		min[0] = 7;
		min[1] = 9;
		max[0] = 50;
		max[1] = 51;
		emin[0] = 32;
		emax[0] = 34;
	}
	else{
		min[0] = 5;
		min[1] = 5;
		max[0] = 50;
		max[1] = 50;
		emin[0] = 22;
		emax[0] = 26;
	}
	multi[0] = 50.0/(emin[0]+emax[0])*2;
 }

float Mag::GetAllign(){
	return 1.0/(v[0*2]+40)-1.0/(v[0*2+1] + 10);
}
