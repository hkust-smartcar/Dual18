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
#include "libbase/k60/pin.h"
#include "config.h"
#include "PID.h"
#include "DualCar_UART.h"

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

bool IsCurve(float front, float mid){
	if (front-mid >= 0.02 || front-mid <= -0.02 || front >= 0.025 || front <= - 0.025){
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
	buzz.SetBeep(false);

	Servo servo(myConfig::GetServoConfig());
	AlternateMotor motorL(myConfig::GetMotorConfig(0));
	motorL.SetClockwise(false);
	AlternateMotor motorR(myConfig::GetMotorConfig(1));
	motorR.SetClockwise(false);
    St7735r lcd(myConfig::GetLcdConfig());
    LcdTypewriter writer(myConfig::GetWriterConfig(&lcd));
    LcdConsole console(myConfig::GetConsoleConfig(&lcd));
    lcd.SetRegion(Lcd::Rect(0,0,128,160));

    DirEncoder LEncoder(myConfig::GetEncoderConfig(1));
    DirEncoder REncoder(myConfig::GetEncoderConfig(0));
    PID servoPIDStraight(2200,0.03);
    PID servoPIDCurve(4825,0.5);//18825,0.5
    PID servoPIDOneStraight(4,100);//10
    PID servoPIDOneCurve(-18,240);//-24,240 turning
    PID servoPIDAlignStraight(-6,1);
    PID servoPIDAlignCurve(-10,0);
    PID motorLPID(0.2,0.0,10, &LEncoder,false);
    PID motorRPID(0.2,0.0,10, &REncoder,true);
//    PID motorLPID(0.145,0.0,1.35, &LEncoder,false);
//    PID motorRPID(0.145,0.0,1.35, &REncoder,true);

    const uint8_t cycle = 8;
    float loopSpeed = 4*cycle, highSpeed = 7*cycle, alignSpeed = 7*cycle;
    float speed = highSpeed;
	float frontLinear, midLinear, diffLinear;
	float raw_frontLinear, raw_midLinear;
	float multiplier = 0.0, top_multiplier = 0.0;
	float front_left = 1, front_right = 1, mid_left = 1, mid_right = 1, back_left = 1, back_right = 1, top_left = 1, top_right = 1;
	uint8_t raw_front_left, raw_front_right, raw_mid_left, raw_mid_right, raw_top_left, raw_top_right;
	float encoderLval, encoderRval;
	float powerL, powerR;
	float pCurve, dCurve, pStraight, dStraight, pMotorL, iMotorL, dMotorL, pMotorR, iMotorR, dMotorR;
	float setAngle = 0;

	typedef enum {
		normal = 0,
		nearLoop,
		straight1,
		straight2,
		turning,
		inLoop,
		junction,
		outLoop,
		leave,
		align,
		side,
		back
	}carState;
	carState state = normal;

	float testLinear = 0;
	bool start = false, cali = false, noCal = true;
	uint8_t approaching = false;
	bool onlyNormal = true, tuningPID = true;
	bool leftLoop = true;
	uint16_t filterSum0 = 0, filterSum1 = 0, filterSum2 = 0, filterSum3 = 0, filterSum4 = 0, filterSum5 = 0;
	uint8_t filterCounter = 0;
	int32_t loopSum = 0;
	uint16_t loopCounter = 0;
	uint32_t lastTime = 0, stateTime = 0, approachTime = 0;

	const uint16_t middleServo = 845, leftServo = 1180, rightServo = 550;
	volatile float angle = middleServo;
	float lastServo = 0;

	uint8_t oneLineMax = 0, oneLineMin = 250, equalMin = 250, equalMax = 0, topMax = 0;
	uint8_t maxLeft = 0,minLeft = 100, maxRight = 0,minRight = 100;
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
    		state = (carState)(((int)state+1) % 12);
    		speed = highSpeed;
    	}
    	else if (State == Joystick::State::kDown){
    		cali = !cali;
    		if (!cali){
				multiplier = 50.0/(equalMin+equalMax)*2;
				top_multiplier = 70.0/topMax;
    		}
    	}
    	else if (State == Joystick::State::kLeft){
    		approaching = !approaching;
    	}
    })));

	DualCar_UART uart0; // << BT related

//	uart0.add(DualCar_UART::FLOAT::f5, &frontLinear, true);
//	uart0.add(DualCar_UART::FLOAT::f1, &midLinear, true);
//	uart0.add(DualCar_UART::FLOAT::f2, &diffLinear, true);

//	uart0.add(DualCar_UART::UINT8_T::u4, &approaching, false);
	uart0.add(DualCar_UART::FLOAT::f4, &speed, true);
//	uart0.add(DualCar_UART::FLOAT::f20, &pCurve, false);
//	uart0.add(DualCar_UART::FLOAT::f21, &dCurve, false);
//	uart0.add(DualCar_UART::FLOAT::f22, &pStraight, false);
//	uart0.add(DualCar_UART::FLOAT::f23, &dStraight, false);
//	uart0.add(DualCar_UART::FLOAT::f24, &pMotorL, false);
//	uart0.add(DualCar_UART::FLOAT::f25, &iMotorL, false);
//	uart0.add(DualCar_UART::FLOAT::f26, &dMotorL, false);
//	uart0.add(DualCar_UART::FLOAT::f27, &pMotorR, false);
//	uart0.add(DualCar_UART::FLOAT::f28, &iMotorR, false);
//	uart0.add(DualCar_UART::FLOAT::f29, &dMotorR, false);

	uart0.add(DualCar_UART::UINT8_T::u0, &approaching, false);
//	uart0.add(DualCar_UART::FLOAT::f4, &multiplier, true);

	uart0.add(DualCar_UART::FLOAT::f0, &encoderLval, true);
	uart0.add(DualCar_UART::FLOAT::f1, &encoderRval, true);
//	uart0.add(DualCar_UART::FLOAT::f2, &powerL, true);
//	uart0.add(DualCar_UART::FLOAT::f3, &powerR, true);
//	uart0.add(DualCar_UART::INT::i0, &encoderLval, true);
//	uart0.add(DualCar_UART::INT::i1, &encoderRval, true);
//	uart0.add(DualCar_UART::INT::i2, &powerL, true);
//	uart0.add(DualCar_UART::INT::i3, &powerR, true);

//	uart0.add(DualCar_UART::FLOAT::f10, &front_left, true); // << BT false:changed by computer
//	uart0.add(DualCar_UART::FLOAT::f11, &front_right, true); // << BT true:changing variable
//	uart0.add(DualCar_UART::FLOAT::f12, &mid_left, true);
//	uart0.add(DualCar_UART::FLOAT::f13, &mid_right, true);
	uart0.add(DualCar_UART::UINT8_T::u13, &equalMin, true);
	uart0.add(DualCar_UART::UINT8_T::u14, &raw_front_left, true);
	uart0.add(DualCar_UART::UINT8_T::u15, &raw_front_right, true);
	uart0.parseValues(); // << BT related

	servo.SetDegree(middleServo);

	if (noCal){
		equalMin = 39;
		equalMax = 39;
		oneLineMin = 6;
		oneLineMax = 80;
		topMax = 70;
		multiplier = 50.0/(equalMin+equalMax)*2;
		top_multiplier = 70.0/topMax;
	}

    while(1){
    	if(System::Time() != lastTime){
    		lastTime = System::Time();

    		uart0.RunEveryMS(); // << BT related

    		if (lastTime % cycle == 0 && filterCounter != 0){
    			raw_mid_left = round(1.0*filterSum0/filterCounter);
    			raw_mid_right = round(1.0*filterSum1/filterCounter);
    			raw_front_left = round(1.0*filterSum2/filterCounter);
    			raw_front_right = round(1.0*filterSum3/filterCounter);
    			raw_top_left = round(1.0*filterSum4/filterCounter);
    			raw_top_right = round(1.0*filterSum5/filterCounter);
    			if (multiplier != 0){
    				mid_left = raw_mid_left*multiplier;
					mid_right = raw_mid_right*multiplier;
					front_left = raw_front_left*multiplier;
					front_right = raw_front_right*multiplier;
					top_left = raw_top_left*top_multiplier;
					top_right = raw_top_right*top_multiplier;
    			}
				filterSum0 = 0;
				filterSum1 = 0;
				filterSum2 = 0;
				filterSum3 = 0;
				filterSum4 = 0;
				filterSum5 = 0;
				filterCounter = 0;
    			if (cali){
					oneLineMin  = min(raw_front_left, min(raw_front_right, min(raw_mid_left, min(raw_mid_right, oneLineMin))));
					oneLineMax  = max(raw_front_left, max(raw_front_right, max(raw_mid_left, max(raw_mid_right, oneLineMax))));
					topMax = max(raw_top_left, max(raw_top_right, topMax));
					if (raw_front_left == raw_front_right){
						if (equalMin > 100 || (equalMin>raw_front_left && equalMin-raw_front_left < 5)){
							equalMin = raw_front_left;

						}
						if (equalMax < 10 || (equalMax<raw_front_left && raw_front_left-equalMax < 5)){
							equalMax = raw_front_left;
						}
					}
    			}

				if (start && (int)state < 8 && max(max(max(raw_front_left,raw_front_right),raw_mid_left),raw_mid_right) < oneLineMin*2){
					start = false;
					motorL.SetPower(0);
					motorR.SetPower(0);
	    			motorLPID.setDesiredVelocity(0);
	    			motorRPID.setDesiredVelocity(0);
				}
				//for alignment
				if (state == normal && approaching){
					state = leave;
					speed = alignSpeed;
				}
				else if (state == leave && raw_front_right < equalMin*1.2 && raw_front_left < equalMin*0.5){
					state = align;
				}
				else if (state == align && (raw_front_right-raw_mid_right)<10 && abs((int)angle-middleServo) < 50){
					state = side;
		    		approachTime = System::Time();
				}
				else if (state == side && !approaching){
					state = back;
					lastServo = -300;
				}
				else if (state == back && raw_front_left>equalMin){
					state = normal;
					speed = highSpeed;
				}

				//nearLoop
				else if (state == normal && IsTwoLine(equalMax, oneLineMax, raw_front_left, raw_front_right) && abs(top_left-top_right)>5){
					state = nearLoop;
					speed = loopSpeed;
					stateTime = lastTime;
					if (top_left>top_right){
						leftLoop = true;
					}else{
						leftLoop = false;
					}
				}
				//straight1
				else if (state == nearLoop && IsTwoLine(equalMax, oneLineMax, raw_mid_left, raw_mid_right)){
					state = straight1;
					stateTime = lastTime;
				}
				//straight2
				else if (state == straight1 && abs(top_left-top_right)<5){
					state = straight2;
					stateTime = lastTime;
				}
				//turning
				else if (state == straight2 && max(top_left,top_right)>15 && ((leftLoop && top_left>top_right) || (!leftLoop && top_right>top_left))){
					state = turning;
					stateTime = lastTime;
				}
				//inLoop
//				if (state == turning && (leftLoop && front_left>front_right*1.45 || !leftLoop && front_right>front_left*1.45) && abs(top_left-top_right)>15){
				else if (state == turning && (leftLoop && raw_front_right<equalMin || !leftLoop && raw_front_left<equalMin)){
					state = inLoop;
					speed = (loopSpeed+highSpeed)/2;
					stateTime = lastTime;
					loopSum = 0;
					loopCounter = 0;
				}
				//junction
				else if (state == inLoop && (top_left>70 || top_right>70)){
					state = junction;
					stateTime = lastTime;
				}
				//outLoop
				else if (state == junction && (top_left<60 && top_right<60)){
					state = outLoop;
					stateTime = lastTime;
				}
				//normal
				else if (state == outLoop && raw_top_left<oneLineMin*2 && raw_top_right<oneLineMin*2){
					state = normal;
					stateTime = lastTime;
					speed = highSpeed;
				}

				raw_frontLinear = 1/(float)raw_front_left-1/(float)raw_front_right;
				raw_midLinear = 1/(float)raw_mid_left-1/(float)raw_mid_right;
				frontLinear = 1/(float)front_left-1/(float)front_right;
				midLinear = 1/(float)mid_left-1/(float)mid_right;
				diffLinear = frontLinear-midLinear;

				if (onlyNormal && ((int)state != 0 && (int)state <= 7)){
					state = normal;
				}

				if (cali){
					angle = setAngle;
				}
				else if (state == normal){
					if (raw_front_left < oneLineMin*2.5 || raw_front_right < oneLineMin*2.5){
						angle = lastServo*1.05;//compare with which side and angle sign
					}
					else{
						if(IsCurve(frontLinear, midLinear)){
							angle = servoPIDCurve.getPID(0.0,frontLinear);
						}else{
							angle = servoPIDStraight.getPID(0.0,frontLinear);
						}
					}
					lastServo = angle;
				}
				else if (state == nearLoop){
					if (leftLoop){
						angle = servoPIDOneStraight.getPID((equalMax+equalMin)/2, raw_mid_right);
					}
					else{
						angle = -servoPIDOneStraight.getPID((equalMax+equalMin)/2, raw_mid_left);
					}
				}
				else if (state == straight1 || state == straight2){
					if (state == straight1){
						angle = servoPIDCurve.getPID(0.0,frontLinear);
					}else{
						angle = servoPIDCurve.getPID(0.0,midLinear);
					}
				}
				else if (state == turning){
					if (leftLoop){
						if (raw_mid_left < (equalMax+equalMin)/2){
							angle = 80;
						}
						else{
							angle = servoPIDOneCurve.getPID((equalMax+equalMin)/2, raw_mid_left);
						}
					}
					else{
						if (raw_mid_right < (equalMax+equalMin)/2){
							angle = -80;
						}
						else{
							angle = -servoPIDOneCurve.getPID((equalMax+equalMin)/2, raw_mid_right);
						}
					}
					lastServo = angle;
				}
				else if (state == inLoop){
					if (raw_front_left < oneLineMin*2.5 || raw_front_right < oneLineMin*2.5){
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
					loopSum += angle;
					loopCounter++;
				}
				else if (state == junction){
					angle = loopSum/loopCounter;
					led3.SetEnable(!0);
					led2.SetEnable(!1);
					led1.SetEnable(!1);
				}
				else if (state == outLoop){
					angle = servoPIDCurve.getPID(0.0,frontLinear);
					led3.SetEnable(!0);
					led2.SetEnable(!1);
					led1.SetEnable(!1);
				}
				else if (state == leave){
					angle = servoPIDAlignCurve.getPID(oneLineMin*2, raw_front_left);
				}
				else if (state == align){
					angle = servoPIDAlignCurve.getPID(equalMin, raw_front_right);
					if (raw_front_left>oneLineMin*3 && angle<0){
						angle = -angle;
					}
				}
				else if (state == side){
					angle = servoPIDAlignStraight.getPID(equalMin, raw_front_right);
				}
				else if (state == back){
//					angle = servoPIDAlignCurve.getPID(equalMax, raw_front_left);
					angle = servoPIDAlignStraight.getPID(equalMax, raw_front_left);
				}

				angle += middleServo;
				angle = max(rightServo,min(leftServo,angle));
				servo.SetDegree(angle);

				if (angle>middleServo){
					motorLPID.setDesiredVelocity(speed*((angle-middleServo)/-7.5+90)/90);
					motorRPID.setDesiredVelocity(speed*((angle-middleServo)/8.5+90)/90);
				}
				else{
					motorRPID.setDesiredVelocity(speed*((middleServo-angle)/-7.5+90)/90);
					motorLPID.setDesiredVelocity(speed*((middleServo-angle)/8.5+90)/90);
				}
//				motorRPID.setDesiredVelocity(speed);
//				motorLPID.setDesiredVelocity(speed);

				encoderLval = LEncoder.GetCount();
				encoderRval = -REncoder.GetCount();

				if (start){
					powerR = motorRPID.getPID();
					powerL = motorLPID.getPID();
					if (powerR > 0){
						motorR.SetClockwise(false);
						motorR.SetPower(powerR);
					}else{
						motorR.SetClockwise(true);
						motorR.SetPower(-powerR);
					}
					if (powerL > 0){
						motorL.SetClockwise(false);
						motorL.SetPower(powerL);
					}else{
						motorL.SetClockwise(true);
						motorL.SetPower(-powerL);
					}

				}else{
					if (encoderLval != 0){
						LEncoder.Update();
					}
					if (encoderRval != 0){
						REncoder.Update();
					}
				}

				if (tuningPID){
//					servoPIDAlignStraight.setkD(dStraight);
//					servoPIDAlignStraight.setkP(pStraight);
//					servoPIDAlignCurve.setkD(dCurve);
//					servoPIDAlignCurve.setkP(pCurve);
//					motorLPID.setkP(pMotorL);
//					motorLPID.setkI(iMotorL);
//					motorLPID.setkD(dMotorL);
//					motorRPID.setkP(pMotorR);
//					motorRPID.setkI(iMotorR);
//					motorRPID.setkD(dMotorR);
				}
			}

			filterCounter++;
			filterSum0 += mag0.GetResult();
			filterSum1 += mag1.GetResult();
			filterSum2 += mag2.GetResult();
			filterSum3 += mag3.GetResult();
			filterSum4 += mag4.GetResult();
			filterSum5 += mag5.GetResult();

			if (lastTime % 500 == 0){
//				if (state != normal && lastTime-stateTime > 5000){
//					state = normal;
//					speed = highSpeed;
//				}
				if (approaching && state == side && lastTime-approachTime>=1000){
					approaching = false;
				}
			}

			if (lastTime % 100 == 0){
				led0.Switch();
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
					sprintf(c,"E: %d %d",equalMax,equalMin);
					writer.WriteBuffer(c,10);
					lcd.SetRegion(Lcd::Rect(0,75,128,15));
					sprintf(c,"M: %d %d", oneLineMin, oneLineMax);
					writer.WriteBuffer(c,10);
					lcd.SetRegion(Lcd::Rect(0,90,128,15));
					sprintf(c,"S: %d %d %d",(int)state, raw_top_left, raw_top_right);
					writer.WriteBuffer(c,10);
					lcd.SetRegion(Lcd::Rect(0,105,128,15));
					sprintf(c,"%d %d %d",(int)top_left, (int)top_right, topMax);
					writer.WriteBuffer(c,10);
					lcd.SetRegion(Lcd::Rect(0,120,128,15));
					sprintf(c,"M: %f",frontLinear*100);
					writer.WriteBuffer(c,10);
					lcd.SetRegion(Lcd::Rect(0,135,128,15));
					sprintf(c,"%d %d %d", servo.GetDegree()-middleServo, equalMin, (int)leftLoop);
					writer.WriteBuffer(c, 10);
				}
			}
    	}
    }
    return 0;
}

