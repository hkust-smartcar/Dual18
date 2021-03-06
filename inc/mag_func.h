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
#include "variable.h"
#include "PID.h"
#include "FlashWrapper.h"

class Mag{
public:
	Mag():
		mag0(myConfig::GetAdcConfig(0)),
		mag1(myConfig::GetAdcConfig(1)),
		mag2(myConfig::GetAdcConfig(2)),
		mag3(myConfig::GetAdcConfig(3)),
		mag4(myConfig::GetAdcConfig(4)){
			mag0.StartConvert();
			mag1.StartConvert();
			mag2.StartConvert();
			mag3.StartConvert();
			mag4.StartConvert();
		};
	typedef enum{
		x_left = 0,
		x_right = 1,
		y_left = 2,
		y_right = 3
	} magPos;
	void TakeSample();
	void Update();
	void Calibrate();
	bool noMagField();
	bool isLoop();
	bool unlikelyCrossRoad();
	bool outLoop();
	bool SmallerThanMin(uint8_t id, float t){return v[id] < min[id]*t*multi[id];}
	bool isBigStraight(){return (xLinear > -0.005 && xLinear < 0.005);};
	void InitMag(uint8_t car_id, FlashWrapper* flash);
	void ResetMag();
	float GetXLinear(){return xLinear;};
	float GetYLinear(){return yLinear;};
	float GetMulti(uint8_t id){return multi[id];};
	uint8_t GetMag(uint8_t id){return v[id];}
	uint8_t GetRaw(uint8_t id){return raw[id];}
	uint8_t GetMin(uint8_t id){return min[id];}
	uint8_t GetMax(uint8_t id){return max[id];}
	uint8_t GetXSum(){return v[magPos::x_left]+v[magPos::x_right];}
	uint8_t GetYSum(){return v[magPos::y_left]+v[magPos::y_right];}
	uint8_t GetXDiff(){return v[magPos::x_left]-v[magPos::x_right];}
	uint8_t GetYDiff(){return v[magPos::y_left]-v[magPos::y_right];}
	bool isTwoLine();
	void CheckState(uint32_t lastTime, uint32_t &approachTime, carState &magState, float &speed, bool &approaching, bool &isFirst, bool &firstArrived, bool &secondArrived, bool &anotherGG, bool &USsent);
	float GetAngle(PID &x_servo, PID &y_servo, PID &align_servo, float &angleX, float &angleY, carState magState, bool left_loop, bool in_loop, float yTarget);
	float GetLastX(){return lastX;}

private:
	Adc mag0;
	Adc mag1;
	Adc mag2;
	Adc mag3;
	Adc mag4;
	uint16_t sum[5] = {0,0,0,0,0};
	uint8_t filterCounter = 0;
	uint8_t v[5] = {255,255,255,255,255}, raw[5] = {255,255,255,255,255};
	float multi[5] = {1.0,1.0,1.0,1.0,1.0};
	float xLinear = 0, yLinear = 0;
	uint8_t min[5] = {15,15,15,15,15}, max[5] = {0,0,0,0,0};
	uint8_t leaveCount = 0;
	float aSpeed = 7, hSpeed = 8;
	float lastX;
};

#endif /* INC_MAG_FUNC_H_ */
