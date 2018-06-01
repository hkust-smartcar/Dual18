/*
 * main.cpp
 *
 * Author: 
 * Copyright (c) 2014-2015 HKUST SmartCar Team
 * Refer to LICENSE for details
 */

#include <cmath>
#include <vector>
#include <cassert>
#include <cstring>
#include <string>
#include <libbase/k60/mcg.h>
#include <libsc/system.h>

#include <libsc/led.h>
#include <libsc/k60/jy_mcu_bt_106.h>
#include "libsc/led.h"
#include "libsc/joystick.h"
#include "libsc/st7735r.h"
#include "libsc/battery_meter.h"
#include "libsc/k60/jy_mcu_bt_106.h"
#include "libbase/k60/pit.h"
#include "libsc/lcd_typewriter.h"
#include "libbase/k60/adc.h"
#include "config.h"
#include "PID.h"
#include "DualCar_UART.h"
//#include "bt.h"

#include "libsc/passive_buzzer.h"

namespace libbase
{
    namespace k60
    {
        Mcg::Config Mcg::GetMcgConfig()
        {
            Mcg::Config config;
            config.external_oscillator_khz = 50000;
            config.core_clock_khz = 150000;
            return config;
        }
    }
}

using libsc::System;
using namespace libsc;
using namespace libbase::k60;
using libsc::k60::JyMcuBt106;


char *str = "";

float inv_sqrt(float x){
	float xhalf = 0.5f*x;
	int i = *(int*)&x;
	i = 0x5f3759df - (i >> 1);
	x = *(float*)&i;
	x = x*(1.5f - xhalf*x*x);
	x = x*(1.5f - xhalf*x*x);
	return x;
}

float _sqrt(float x){
	return x*inv_sqrt(x);
}

int max(int a, int b){return a>b?a:b;}
int min(int a, int b){return a<b?a:b;}

bool IsOneLine(uint8_t maxVal, uint8_t leftVal, uint8_t rightVal, float multi = 1.5){
	if (max(leftVal,rightVal) < maxVal && (leftVal+rightVal) < maxVal*multi){
		return true;
	}
	return false;
}

bool IsTwoLine(uint8_t midMax, uint8_t sideMax, uint8_t leftVal, uint8_t rightVal){
	if (max(leftVal,rightVal) > sideMax*1.05 || min(leftVal,rightVal) > midMax*1.15){
		return true;
	}
	return false;
}

int main() {


    System::Init();

    Led led0(myConfig::GetLedConfig(0));
    Led led1(myConfig::GetLedConfig(1));
    Led led2(myConfig::GetLedConfig(2));
    Led led3(myConfig::GetLedConfig(3));

    led0.SetEnable(1);
    led1.SetEnable(1);
    led2.SetEnable(1);
    led3.SetEnable(1);

	Adc mag0(myConfig::GetAdcConfig(0));
	Adc mag1(myConfig::GetAdcConfig(1));
	Adc mag2(myConfig::GetAdcConfig(2));
	Adc mag3(myConfig::GetAdcConfig(3));
	Adc mag4(myConfig::GetAdcConfig(4));
	Adc mag5(myConfig::GetAdcConfig(5));

	mag0.StartConvert();
	mag1.StartConvert();
	mag2.StartConvert();
	mag3.StartConvert();

	PassiveBuzzer::Config config;
	PassiveBuzzer buzz(config);
	buzz.SetNote(523);
//	buzz.SetBeep(true);

	Servo servo(myConfig::GetServoConfig());
	AlternateMotor motorL(myConfig::GetMotorConfig(0));
	motorL.SetClockwise(false);
	AlternateMotor motorR(myConfig::GetMotorConfig(1));
    St7735r lcd(myConfig::GetLcdConfig());
    LcdTypewriter writer(myConfig::GetWriterConfig(&lcd));
    LcdConsole console(myConfig::GetConsoleConfig(&lcd));
    lcd.SetRegion(Lcd::Rect(0,0,128,160));

    DirEncoder LEncoder(myConfig::GetEncoderConfig(1));
    DirEncoder REncoder(myConfig::GetEncoderConfig(0));
//    PID servoPIDCurv(6275,69);
    PID servoPIDStraight(2200,0.03);
    PID servoPIDCurve(4825,0.5);//18825,0.5
    PID servoPIDOneStraight(4,100);//10
    PID servoPIDOneCurve(-18,240);//-24,240
    PID motorLPID(0.145,0.0,1.35, &LEncoder);
    PID motorRPID(0.145,0.0,1.35, &REncoder);
    const uint8_t cycle = 8;
    float lowSpeed = 5*cycle, highSpeed = 7*cycle;
    float speed = highSpeed;
	float lastServo, frontLinear, midLinear, diffLinear;
	float raw_frontLinear, raw_midLinear;
	float multiplier = 0.0;
	float front_left = 1, front_right = 1, mid_left = 1, mid_right = 1, back_left = 1, back_right = 1;
	uint8_t raw_front_left, raw_front_right, raw_mid_left, raw_mid_right, raw4, raw5;
	uint8_t encoderLval, encoderRval, powerL, powerR;
	float pCurve, dCurve, pStraight, dStraight, pMotor, iMotor, dMotor;
//    bt mBT(&servoPIDCurv, &servoPIDStraight, &motorLPID, &motorRPID, &speed, &frontLinear);

	typedef enum {
		normal = 0,
		nearLoop,
		straight1,
		straight2,
		turning,
		inLoop,
		outLoop
	}carState;
	carState state = normal;

	bool start = false;
	bool onlyNormal = false, tuningPID = false;
	bool leftLoop = true, bigVal = false;
	uint16_t filterSum0 = 0, filterSum1 = 0, filterSum2 = 0, filterSum3 = 0, filterSum4 = 0, filterSum5 = 0;
	uint8_t filterCounter = 0;
	uint32_t lastTime = 0, stateTime = 0;

	const uint16_t middleServo = 865, leftServo = 1180, rightServo = 550;
	float angle = middleServo;

	uint8_t oneLineMax = 0, oneLineMin = 250, equalMin = 250, equalMax = 0;
	uint8_t maxLeft = 0,minLeft = 100,maxRight = 0,minRight = 100;
	uint8_t count1, count4;

    Joystick js(myConfig::GetJoystickConfig(Joystick::Listener([&](const uint8_t id, const Joystick::State State){
    	if (State == Joystick::State::kSelect){
    		if (start){
    			start = false;
    			motorLPID.setDesiredVelocity(0);
    			motorRPID.setDesiredVelocity(0);
				motorL.SetPower(0);
				motorR.SetPower(0);
    		}
    		else{
				start = true;
				motorLPID.setDesiredVelocity(speed);
				motorRPID.setDesiredVelocity(speed);
    		}
		}
    	else if (State == Joystick::State::kUp){
    		state = (carState)(((int)state+1) % 7);
    	}
    })));

	DualCar_UART uart0; // << BT related

	uart0.add(DualCar_UART::FLOAT::f0, &frontLinear, true);
	uart0.add(DualCar_UART::FLOAT::f1, &midLinear, true);
	uart0.add(DualCar_UART::FLOAT::f2, &diffLinear, true);

	uart0.add(DualCar_UART::FLOAT::f3, &speed, false);
	uart0.add(DualCar_UART::FLOAT::f20, &pCurve, false);
	uart0.add(DualCar_UART::FLOAT::f21, &dCurve, false);
	uart0.add(DualCar_UART::FLOAT::f22, &pStraight, false);
	uart0.add(DualCar_UART::FLOAT::f23, &dStraight, false);
//	uart0.add(DualCar_UART::FLOAT::f24, &pMotor, false);
//	uart0.add(DualCar_UART::FLOAT::f25, &iMotor, false);
//	uart0.add(DualCar_UART::FLOAT::f26, &dMotor, false);

	uart0.add(DualCar_UART::FLOAT::f4, &multiplier, true);

	uart0.add(DualCar_UART::UINT8_T::u0, &encoderLval, true);
	uart0.add(DualCar_UART::UINT8_T::u1, &encoderRval, true);
//	uart0.add(DualCar_UART::UINT8_T::u2, &powerL, true);
//	uart0.add(DualCar_UART::UINT8_T::u3, &powerR, true);

	uart0.add(DualCar_UART::FLOAT::f10, &front_left, true); // << BT false:changed by computer
	uart0.add(DualCar_UART::FLOAT::f11, &front_right, true); // << BT true:changing variable
	uart0.add(DualCar_UART::FLOAT::f12, &mid_left, true);
	uart0.add(DualCar_UART::FLOAT::f13, &mid_right, true);
	uart0.add(DualCar_UART::UINT8_T::u14, &raw_front_left, true); // << BT false:changed by computer
	uart0.add(DualCar_UART::UINT8_T::u15, &raw_front_right, true); // << BT true:changing variable
	uart0.parseValues(); // << BT related

	servo.SetDegree(middleServo);
    while(1){
    	if(System::Time() != lastTime){
    		lastTime = System::Time();

    		uart0.RunEveryMS(); // << BT related

    		if (lastTime % cycle == 0){
    			raw_mid_left = round(1.0*filterSum0/filterCounter);
    			raw_mid_right = round(1.0*filterSum1/filterCounter);
    			raw_front_left = round(1.0*filterSum2/filterCounter);
    			raw_front_right = round(1.0*filterSum3/filterCounter);
    			raw4 = round(1.0*filterSum4/filterCounter);
    			raw5 = round(1.0*filterSum5/filterCounter);
    			if (multiplier != 0){
//    				mid_left = (raw_mid_left-oneLineMin)*multiplier;
//					mid_right = (raw_mid_right-oneLineMin)*multiplier;
//					front_left = (raw_front_left-oneLineMin)*multiplier;
//					front_right = (raw_front_right-oneLineMin)*multiplier;
    				mid_left = raw_mid_left*multiplier;
					mid_right = raw_mid_right*multiplier;
					front_left = raw_front_left*multiplier;
					front_right = raw_front_right*multiplier;
    			}
				filterSum0 = 0;
				filterSum1 = 0;
				filterSum2 = 0;
				filterSum3 = 0;
				filterSum4 = 0;
				filterSum5 = 0;
				filterCounter = 0;
    			if (lastTime < 10000){
					oneLineMin  = min(raw_front_left, min(raw_front_right, min(raw_mid_left, min(raw_mid_right, oneLineMin))));
					oneLineMax  = max(raw_front_left, max(raw_front_right, max(raw_mid_left, max(raw_mid_right, oneLineMax))));
					if (raw_front_left == raw_front_right){
						if (equalMin > 100 || (equalMin>raw_front_left && equalMin-raw_front_left < 5)){
							equalMin = raw_front_left;

						}
						if (equalMax < 10 || (equalMax<raw_front_left && raw_front_left-equalMax < 5)){
							equalMax = raw_front_left;
						}
					}
    			}else if (lastTime < 10010 || multiplier == 0){
					multiplier = 50.0/(equalMin+equalMax)*2;//-2*oneLineMin
    			}

				if (start && max(max(max(raw_front_left,raw_front_right),raw_mid_left),raw_mid_right) < oneLineMin*1.5){
					start = false;
					state = normal;
					motorL.SetPower(0);
					motorR.SetPower(0);
	    			motorLPID.setDesiredVelocity(0);
	    			motorRPID.setDesiredVelocity(0);
				}

				if (!IsTwoLine(equalMax, oneLineMax, raw_front_left, raw_front_right)){
					buzz.SetBeep(false);
					if (state == turning && (leftLoop && front_left>front_right*1.45 || !leftLoop && front_right>front_left*1.45)){
						state = inLoop;
						speed = (lowSpeed+highSpeed)/2;
						stateTime = lastTime;
						bigVal = false;
					}
//					else if (IsOneLine(oneLineMax, raw_front_left, raw_front_right) && IsOneLine(oneLineMax, raw_mid_left, raw_mid_right) && (state == inLoop && bigVal || state == outLoop)){
					else if (!IsTwoLine(equalMax, oneLineMax, raw_mid_left, raw_mid_right) && (state == inLoop && bigVal || state == outLoop)){//both is one line
						state = normal;
						stateTime = lastTime;
						speed = highSpeed;
					}
				}
				else{//front is two line
//					buzz.SetBeep(lastTime >= 10000);
					if (state == normal){
						state = nearLoop;
						speed = lowSpeed;
						stateTime = lastTime;
						if (front_left-mid_left > front_right-mid_right){
							leftLoop = true;
						}
						else{
							leftLoop = false;
						}
					}
					if (IsTwoLine(equalMax, oneLineMax, raw_mid_left, raw_mid_right)){//both are two line
						if (state == nearLoop){
							state = straight1;
							stateTime = lastTime;
						}
						else if (state == inLoop){
							state = outLoop;
							stateTime = lastTime;
						}
						else if (state == straight1 && (raw_mid_left+raw_mid_right)>(raw_front_left+raw_front_right) && abs(raw_mid_left-raw_mid_right)<abs(raw_front_left-raw_front_right)){
							state = straight2;
							stateTime = lastTime;
						}
						else if (state == straight2 && (leftLoop && front_left*0.9>mid_left  && mid_right*0.9>front_right || !leftLoop && front_right*0.9>mid_right && mid_left*0.9>front_left)){//0.9
							state = turning;
							stateTime = lastTime;
						}
					}
				}

				raw_frontLinear = 1/(float)raw_front_left-1/(float)raw_front_right;
				raw_midLinear = 1/(float)raw_mid_left-1/(float)raw_mid_right;
				frontLinear = 1/(float)front_left-1/(float)front_right;
				midLinear = 1/(float)mid_left-1/(float)mid_right;
				diffLinear = frontLinear-midLinear;

				if (onlyNormal && state != normal){
					state = normal;
				}
				if (state == normal){
					if (raw_front_left < oneLineMin*2 || raw_front_right < oneLineMin*2){
						angle = lastServo*1.3;
					}
					else{
//						if(frontLinear-midLinear >= 0.01 || frontLinear-midLinear <= -0.01 || frontLinear >= 0.035 || frontLinear <= - 0.035){//0.003
						if(frontLinear-midLinear >= 0.02 || frontLinear-midLinear <= -0.02 || frontLinear >= 0.025 || frontLinear <= - 0.025){//0.003
							angle = servoPIDCurve.getPID(0.0,frontLinear);
							if (onlyNormal){led0.SetEnable(!0);}
						}else{
							angle = servoPIDStraight.getPID(0.0,frontLinear);
							if (onlyNormal){led0.SetEnable(!1);}
						}
						lastServo = angle;
					}
					led3.SetEnable(!0);
					led2.SetEnable(!0);
					led1.SetEnable(!0);
				}
				else if (state == nearLoop){
					if (leftLoop){
						angle = servoPIDOneStraight.getPID((equalMax+equalMin)/2, raw_mid_right);
					}
					else{
						angle = -servoPIDOneStraight.getPID((equalMax+equalMin)/2, raw_mid_left);
					}
					led3.SetEnable(!1);
					led2.SetEnable(!0);
					led1.SetEnable(!0);
				}
				else if (state == straight1 || state == straight2){
					if (state == straight1){
						angle = servoPIDCurve.getPID(0.0,frontLinear);
					}else{
						angle = servoPIDCurve.getPID(0.0,midLinear);
					}
					led3.SetEnable(!(state == straight2));
					led2.SetEnable(!1);
					led1.SetEnable(!0);
				}
				else if (state == turning){
					if (leftLoop){
						if (raw_mid_left < (equalMax+equalMin)/2){
							angle = 100;
						}
						else{
							angle = servoPIDOneCurve.getPID((equalMax+equalMin)/2, raw_mid_left);
						}
					}
					else{
						if (raw_mid_right < (equalMax+equalMin)/2){
							angle = -100;
						}
						else{
							angle = -servoPIDOneCurve.getPID((equalMax+equalMin)/2, raw_mid_right);
						}
					}
					lastServo = angle;
					led3.SetEnable(!0);
					led2.SetEnable(!0);
					led1.SetEnable(!1);
				}
				else if (state == inLoop){
					if (!bigVal){
						if (raw_front_left < oneLineMin*2 || raw_front_right < oneLineMin*2){
							if (leftLoop){
								angle = 300;
							}
							else{
								angle = -300;
							}
						}
						else{
							angle = servoPIDCurve.getPID(0.0,frontLinear);
							lastServo = angle;
						}
						if (max(raw_front_left, raw_front_right) > oneLineMax){
							bigVal = true;
						}
					}
					else{
						if (raw_front_left < equalMax || raw_front_right < equalMax){//equalMax may not be the best value, 50
							if (leftLoop){
								angle = 300;
							}
							else{
								angle = -300;
							}
						}
						else{
							angle = servoPIDCurve.getPID(0.0,frontLinear);
						}
					}
					led3.SetEnable(!1);
					led2.SetEnable(!0);
					led1.SetEnable(!1);
				}
				else if (state == outLoop){
					if (raw_front_left < oneLineMin*2 || raw_front_right < oneLineMin*2){
						if (leftLoop){
							angle = 300;
						}
						else{
							angle = -300;
						}
					}
					else{
						angle = servoPIDCurve.getPID(0.0,frontLinear);
						lastServo = angle;
					}
					led3.SetEnable(!0);
					led2.SetEnable(!1);
					led1.SetEnable(!1);
				}
				if (!onlyNormal){
					led0.SetEnable(!leftLoop);
				}

				angle += middleServo;
				angle = max(rightServo,min(leftServo,angle));
				servo.SetDegree(angle);

//				if (angle>middleServo){
//					motorLPID.setDesiredVelocity(speed*((angle-middleServo)/-7.5+90)/90);
//					motorRPID.setDesiredVelocity(speed*((angle-middleServo)/8.5+90)/90);
//				}
//				else{
//					motorRPID.setDesiredVelocity(speed*((middleServo-angle)/-7.5+90)/90);
//					motorLPID.setDesiredVelocity(speed*((middleServo-angle)/8.5+90)/90);
//				}
				motorRPID.setDesiredVelocity(speed);
				motorLPID.setDesiredVelocity(speed);

				if (start){
					powerR = motorRPID.getPID();
					powerL = motorLPID.getPID();
					motorR.SetPower(powerR);
					motorL.SetPower(powerL);
				}

				encoderLval = LEncoder.GetCount();
				encoderRval = REncoder.GetCount();
				if (tuningPID){
					servoPIDStraight.setkD(dStraight);
					servoPIDStraight.setkP(pStraight);
					servoPIDCurve.setkD(dCurve);
					servoPIDCurve.setkP(pCurve);
				}
//				motorLPID.setkP(pMotor);
//				motorLPID.setkI(iMotor);
//				motorLPID.setkD(dMotor);
//				mBT.sendVelocity();
			}

			filterCounter++;
			filterSum0 += mag0.GetResult();
			filterSum1 += mag1.GetResult();
			filterSum2 += mag2.GetResult();
			filterSum3 += mag3.GetResult();
			filterSum4 += mag4.GetResult();
			filterSum5 += mag5.GetResult();



			if (lastTime % 500 == 0){
				if (state != normal && lastTime-stateTime > 30000){
					state = normal;
					speed = highSpeed;
				}
			}

			if (lastTime % 100 == 0){
				if(!start){
					char c[10];
					lcd.SetRegion(Lcd::Rect(0,0,128,15));
					sprintf(c,"rFL: %d", raw_front_left);
					writer.WriteBuffer(c,10);
					lcd.SetRegion(Lcd::Rect(0,15,128,15));
					sprintf(c,"rFR: %d", raw_front_right);
					writer.WriteBuffer(c,10);
					lcd.SetRegion(Lcd::Rect(0,30,128,15));
					sprintf(c,"rML: %d",raw_mid_left);
					writer.WriteBuffer(c,10);
					lcd.SetRegion(Lcd::Rect(0,45,128,15));
					sprintf(c,"rMR: %d",raw_mid_right);
					writer.WriteBuffer(c,10);
					lcd.SetRegion(Lcd::Rect(0,60,128,15));
					sprintf(c,"P: %d %d",equalMax,equalMin);
					writer.WriteBuffer(c,10);
					lcd.SetRegion(Lcd::Rect(0,75,128,15));
					sprintf(c,"M: %d %d", oneLineMin, oneLineMax);
					writer.WriteBuffer(c,10);
					lcd.SetRegion(Lcd::Rect(0,90,128,15));
					sprintf(c,"A: %d",servo.GetDegree());
					writer.WriteBuffer(c,10);
					lcd.SetRegion(Lcd::Rect(0,105,128,15));
					sprintf(c,"%d %d",raw4, raw5);
					writer.WriteBuffer(c,10);
					lcd.SetRegion(Lcd::Rect(0,120,128,15));
					sprintf(c,"M: %f",frontLinear*100);
					writer.WriteBuffer(c,10);
////					if (state == normal){
////						lcd.FillColor(0xFF00);
////					}else if (state == nearLoop){
////						if (isLeft){
////							lcd.SetRegion(Lcd::Rect(0,120,64,15));
////						}
////						else{
////							lcd.SetRegion(Lcd::Rect(64,120,64,15));
////						}
////						lcd.FillColor(0x0FF0);
////					}else if (state == turning){
////						lcd.FillColor(0x00FF);
////					}else if (state == inLoop){
////						lcd.FillColor(0x0000);
////					}
					lcd.SetRegion(Lcd::Rect(0,135,128,15));
					sprintf(c,"M: %f",midLinear*100);
					writer.WriteBuffer(c, 10);
				}
			}
    	}
    }
    return 0;
}

