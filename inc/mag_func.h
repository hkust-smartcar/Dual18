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

class Mag{
public:
	Mag():
		mag0(myConfig::GetAdcConfig(0)),
		mag1(myConfig::GetAdcConfig(1)),
		mag2(myConfig::GetAdcConfig(2)),
		mag3(myConfig::GetAdcConfig(3)),
		mag4(myConfig::GetAdcConfig(4)),
		mag5(myConfig::GetAdcConfig(5)){

		mag0.StartConvert();
		mag1.StartConvert();
		mag2.StartConvert();
		mag3.StartConvert();
		mag4.StartConvert();
		mag5.StartConvert();
	};
	typedef enum{
		x_left = 1,
		x_right = 5,
		y_left = 0,
		y_right = 4
	} magPos;
	void TakeSample();
	void Update();
	void Calibrate();
	bool noMagField();
	bool isLoop();
	bool unlikelyCrossRoad();
	bool outLoop();
	bool isMidLoop();
	bool SmallerThanMin(uint8_t id, float t){return v[id] < min[id]*t*multi[id];}
	bool isBigStraight(){return (xLinear > -0.005 && xLinear < 0.005);};
	void InitMag(uint8_t car_id);
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
	void CheckState(uint32_t &lastTime, uint32_t &approachTime, carState &magState, float &speed, bool &approaching, bool &isFirst, bool &firstArrived, bool &secondArrived);
	float GetAngle(PID &x_servo, PID &y_servo, PID &align_servo, float &angleX, float &angleY, carState &magState, bool &left_loop);

private:
	Adc mag0;
	Adc mag1;
	Adc mag2;
	Adc mag3;
	Adc mag4;
	Adc mag5;
	uint16_t sum[6] = {0,0,0,0,0,0};
	uint8_t filterCounter = 0;
	uint8_t v[6] = {255,255,255,255,255,255}, raw[6] = {255,255,255,255,255,255};
	float multi[6] = {1.0,1.0,1.0,1.0,1.0,1.0};
	float xLinear = 0, yLinear = 0;
	uint8_t min[6] = {15,15,15,15,15,15}, max[6] = {0,0,0,0,0,0};
	uint8_t leaveCount = 0;
	float aSpeed = 9, hSpeed = 9;
};

#endif /* INC_MAG_FUNC_H_ */
