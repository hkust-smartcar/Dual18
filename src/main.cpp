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
#include "bt.h"
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

	mag0.StartConvert();
	mag1.StartConvert();
	mag2.StartConvert();
	mag3.StartConvert();

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
    PID servoPIDCurve(500, 14000);
    PID servoPIDStraight(250,12000);
    PID motorLPID(0.32,0.0,8, &LEncoder);
    PID motorRPID(0.32,0.0,8, &REncoder);
    float speed = 30;
	float magRatio, xRatio, xRatio2;
    bt mBT(&servoPIDCurve, &servoPIDStraight, &motorLPID, &motorRPID, &speed, &xRatio);

	typedef enum {
		normal = 0,
		nearLoop,
		straight,
		turning,
		inLoop
	}carState;
	carState state = normal;
	bool start = false;
	//
	uint16_t filterSum0 = 0, filterSum1 = 0, filterSum2 = 0, filterSum3 = 0;
	uint8_t filterCounter = 0;
    //
	uint32_t lastTime = 0;
    uint32_t greenTime = 0;
	bool isLeft = 1;
	bool isCurve = 0;
	uint8_t left_mag, right_mag, mid_left, mid_right;
	float angle = 0;

	int magSum = 0;
	uint16_t middleServo = 830;
	uint16_t leftServo = 1200;
	uint16_t rightServo = 500;
	bool diff = true;
    Joystick js(myConfig::GetJoystickConfig(Joystick::Listener([&](const uint8_t id, const Joystick::State state){
    	if (state == Joystick::State::kSelect){
    		if (start){
    			start = false;
    			motorLPID.setDesiredVelocity(0);
    			motorRPID.setDesiredVelocity(0);
    		}
    		else{
				start = true;
				motorLPID.setDesiredVelocity(speed);
				motorRPID.setDesiredVelocity(speed);
    		}
		}
		else if (state == Joystick::State::kSelect){
			diff = !diff;
		}
    	led2.Switch();
    })));

	servo.SetDegree(middleServo);
    while(1){
    	if(System::Time() != lastTime){
    		lastTime = System::Time();
		if (lastTime % 6== 0){
				//
				left_mag = filterSum0/filterCounter;
				right_mag = filterSum1/filterCounter;
				mid_left = filterSum2/filterCounter;
				mid_right = filterSum3/filterCounter;
				filterSum0 = 0;
				filterSum1 = 0;
				filterSum2 = 0;
				filterSum3 = 0;
				filterCounter = 0;
				//
				magSum = left_mag + right_mag;

				xRatio = (float)(right_mag - left_mag)/(magSum);
				xRatio2 = (float)(mid_right - mid_left)/(mid_right + mid_left);
				if(xRatio2 >= 0.2 || xRatio2 <= -0.2 || xRatio >= 0.2 || xRatio <= -0.2){
					isCurve = true;
				}else{
					isCurve = false;
				}
				if(isCurve){
					angle = servoPIDCurve.getPID(0.0,xRatio);
				}else{
					angle = servoPIDStraight.getPID(0.0,xRatio);
				}
				angle += middleServo;
				angle = max(rightServo,min(leftServo,angle));
//				if(state == normal && magSum > 230){
//					state = nearLoop;
//					if(right_mag > left_mag){
//						isLeft = false;
//					}else{
//						isLeft = true;
//					}
//				}else if(state == nearLoop && magSum < 215){
//					state = turning;
//				}else if(state == turning && magSum < 150){
//					if(isLeft){
//						if(angle < middleServo + 100){
//							state = inLoop;
//						}
//					}else{
//						if(angle > middleServo - 100){
//							state = inLoop;
//						}
//					}
//
//				}

				if(state == normal || state == nearLoop || state == inLoop){
					servo.SetDegree(angle);
				}
//				if(state == turning){
//					int offset = 30;
//					if(isLeft){
//						servo.SetDegree(leftServo - offset);
//					}else{
//						servo.SetDegree(rightServo + offset);
//					}
//				}
				if (diff){
					if (angle>middleServo){
						motorLPID.setDesiredVelocity(speed*((angle-middleServo)/-7.5+90)/90);
						motorRPID.setDesiredVelocity(speed*((angle-middleServo)/8.5+90)/90);
					}
					else{
						motorRPID.setDesiredVelocity(speed*((angle-middleServo)/-7.5+90)/90);
						motorLPID.setDesiredVelocity(speed*((angle-middleServo)/8.5+90)/90);
					}
				}
				if (start){
					motorL.SetPower(motorLPID.getPID());
					motorR.SetPower(motorRPID.getPID());
				}
				mBT.sendVelocity();
			}

			//
			filterCounter++;
			filterSum0 += mag0.GetResult();
			filterSum1 += mag1.GetResult();
			filterSum2 += mag2.GetResult();
			filterSum3 += mag3.GetResult();
			//

			if (lastTime % 100 == 0){
				if(!start){
					char c[10];
					lcd.SetRegion(Lcd::Rect(0,0,128,15));
					sprintf(c,"L: %d", left_mag);
					writer.WriteBuffer(c,10);
					lcd.SetRegion(Lcd::Rect(0,15,128,15));
					sprintf(c,"R: %d", right_mag);
					writer.WriteBuffer(c,10);
					lcd.SetRegion(Lcd::Rect(0,30,128,15));
					sprintf(c,"ML: %d",mid_left);
					writer.WriteBuffer(c,10);
					lcd.SetRegion(Lcd::Rect(0,45,128,15));
					sprintf(c,"MR: %d",mid_right);
					writer.WriteBuffer(c,10);
					lcd.SetRegion(Lcd::Rect(0,60,128,15));
					sprintf(c,"SUM: %d",left_mag + right_mag);
					writer.WriteBuffer(c,10);
					lcd.SetRegion(Lcd::Rect(0,75,128,15));
					sprintf(c,"d: %d",servo.GetDegree());
//					sprintf(c,"d: %f",servoPID.getdTerm());
					writer.WriteBuffer(c,10);
					lcd.SetRegion(Lcd::Rect(0,90,128,15));
					sprintf(c,"L: %f",motorLPID.getcurrentVelocity());
					writer.WriteBuffer(c,10);
					lcd.SetRegion(Lcd::Rect(0,105,128,15));
					sprintf(c,"R: %f",motorRPID.getcurrentVelocity());
					writer.WriteBuffer(c,10);
					lcd.SetRegion(Lcd::Rect(0,120,128,15));
					if (state == normal){
						lcd.FillColor(0xFF00);
					}else if (state == nearLoop){
						if (isLeft){
							lcd.SetRegion(Lcd::Rect(0,120,64,15));
						}
						else{
							lcd.SetRegion(Lcd::Rect(64,120,64,15));
						}
						lcd.FillColor(0x0FF0);
					}else if (state == turning){
						lcd.FillColor(0x00FF);
					}else if (state == inLoop){
						lcd.FillColor(0x0000);
					}
					lcd.SetRegion(Lcd::Rect(0,135,128,15));
					sprintf(c,"xR: %f", xRatio - xRatio2);
					writer.WriteBuffer(c, 10);
				}
			}
    	}
    }
    return 0;
}

