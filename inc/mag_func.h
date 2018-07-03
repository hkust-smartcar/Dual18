/*
 * mag_func.h
 *
 *  Created on: 21 Jun 2018
 *      Author: PWS
 */

#ifndef INC_MAG_FUNC_H_
#define INC_MAG_FUNC_H_

#include "libbase/k60/adc.h"
#include "config.h"

class Mag{
public:
	Mag(bool isCar1):
		mag0(myConfig::GetAdcConfig(0, isCar1)),
		mag1(myConfig::GetAdcConfig(1, isCar1)),
		mag2(myConfig::GetAdcConfig(2, isCar1)),
		mag3(myConfig::GetAdcConfig(3, isCar1)),
		mag4(myConfig::GetAdcConfig(4, isCar1)),
		mag5(myConfig::GetAdcConfig(5, isCar1)){
		mag0.StartConvert();
		mag1.StartConvert();
		mag2.StartConvert();
		mag3.StartConvert();
		mag4.StartConvert();
		mag5.StartConvert();
	};
	void TakeSample();
	void Update();
	void Calibrate();
	bool noMagField();
	bool SmallerThanE(uint8_t id, float t){return v[id] < emin[id/2]*t*multi[id/2];}
	bool SmallerThanMin(uint8_t id, float t){return v[id] < min[id]*t*multi[id/2];}
	bool BigMag(){return ((v[0]+v[1] > 125) && (v[0] > 65 || v[1] > 65));};
	bool isBigStraight(){return (linear[0] > -0.005 && linear[0] < 0.005);};
	int Difference(uint8_t id0, uint8_t id1){return v[id0]-v[id1];}
	void SetMag(uint8_t id);
	float GetLinear(uint8_t pair_id);
	float GetMulti(uint8_t pair_id);
	uint8_t GetMag(uint8_t id){return v[id];}
	uint8_t GetMin(uint8_t id){return min[id];}
	uint8_t GetMax(uint8_t id){return max[id];}
	uint8_t GetEMin(uint8_t id){return emin[id];}
	uint8_t GetEMax(uint8_t id){return emax[id];}

private:
	Adc mag0;
	Adc mag1;
	Adc mag2;
	Adc mag3;
	Adc mag4;
	Adc mag5;
	uint16_t sum[6] = {0};
	uint8_t filterCounter = 0;
	uint8_t v[6] ={255};
	float multi[3] = {1.0}, linear[3] = {0};
	uint8_t min[6] = {15}, max[6] = {0},
			emin[3] = {255}, emax[3] = {0};
};

#endif /* INC_MAG_FUNC_H_ */
