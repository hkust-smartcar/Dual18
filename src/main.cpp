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

bool IsOneLine(uint8_t maxVal, uint8_t leftVal, uint8_t rightVal, float multi = 1.3){
	if (max(leftVal,rightVal) < maxVal && (leftVal+rightVal) < maxVal*multi){
		return true;
	}
	return false;
}

bool IsTwoLine(uint8_t maxVal, uint8_t leftVal, uint8_t rightVal, float multi = 1.3){
	if (max(leftVal,rightVal) > maxVal && (leftVal+rightVal) > maxVal*multi){
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
    PID servoPIDCurv(5800, 110000);
    PID servoPIDStraight(2500,80000);
    PID servoPIDCurve(3000, 110000);//for straight123
    PID servoPIDOneStraight(4,100);//10
    PID servoPIDOneCurve(-26,220);
    PID motorLPID(0.38,0.0,1, &LEncoder);
    PID motorRPID(0.38,0.0,1, &REncoder);
    float speed = 14;
	float lastServo, frontLinear, midLinear;
    bt mBT(&servoPIDCurv, &servoPIDStraight, &motorLPID, &motorRPID, &speed, &frontLinear);

	typedef enum {
		normal = 0,
		nearLoop,
		straight1,
		straight2,
		straight3,
		turning,
		inLoop,
		outLoop
	}carState;
	carState state = normal;
	bool start = false;
	bool leftLoop = true, bigVal = false;
	uint16_t filterSum0 = 0, filterSum1 = 0, filterSum2 = 0, filterSum3 = 0, filterSum4 = 0, filterSum5 = 0;
	uint8_t filterCounter = 0;
	uint32_t lastTime = 0;
	uint8_t front_left, front_right, mid_left, mid_right, back_left, back_right;

	uint16_t middleServo = 865;
	uint16_t leftServo = 1180;
	uint16_t rightServo = 550;
	float angle = middleServo;

	bool tuneP = false, goLoop = true;
	uint8_t oneLineMax = 90, equalMin = 55, equalMax = 51;//swapped min and max
	uint8_t maxLeft = 0,minLeft = 100,maxRight = 0,minRight = 100;

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
			goLoop = !goLoop;
		}
    	else{
    		state = (carState)(((int)state+1) % 8);
    	}
    	led2.Switch();
    })));

	servo.SetDegree(middleServo);
    while(1){
    	if(System::Time() != lastTime){
    		lastTime = System::Time();
			mid_left = mag0.GetResult();
			mid_right = mag1.GetResult();
			front_left = mag2.GetResult();
			front_right = mag3.GetResult();
			if (!IsTwoLine(oneLineMax, front_left, front_right) ){
				if (state == turning && (leftLoop && front_left>front_right*1.5 || !leftLoop && front_right>front_left*1.5)){
					state = inLoop;
					bigVal = false;
				}
				else if (IsOneLine(oneLineMax, front_left, front_right) && IsOneLine(oneLineMax, mid_left, mid_right) && (state == inLoop && bigVal || state == outLoop)){
					state = normal;
				}
			}
			else{
				if (!IsTwoLine(oneLineMax, mid_left, mid_right)){
					if (state == normal){
						state = nearLoop;
						if (front_left > front_right){
							leftLoop = true;
						}
						else{
							leftLoop = false;
						}
					}
				}
				else{
					if (state == nearLoop){
						state = straight1;
					}
					else if (state == inLoop){
						state = outLoop;
					}
					else if (state == straight1 && abs(mid_left-mid_right) < 15){
						state = straight2;
					}
					else if (state == straight2 && abs(mid_left-mid_right) > 15){
						state = straight3;
					}
					else if (state == straight3 && (leftLoop && front_left*0.85>front_right || !leftLoop && front_right*0.85>front_left)){//originally *0.85
						state = turning;
					}
				}
			}

    		if (lastTime % 4 == 0){
				mid_left = filterSum0/filterCounter;
				mid_right = filterSum1/filterCounter;
				front_left = filterSum2/filterCounter;
				front_right = filterSum3/filterCounter;
				back_left = filterSum4/filterCounter;
				back_right = filterSum5/filterCounter;
				if (filterCounter > 15){
					filterSum0 = 0;
					filterSum1 = 0;
					filterSum2 = 0;
					filterSum3 = 0;
					filterSum4 = 0;
					filterSum5 = 0;
					filterCounter = 0;
				}
				minLeft = min(minLeft, front_left);
				maxLeft = max(maxLeft, front_left);
				minRight = min(minRight, front_right);
				maxRight = max(maxRight, front_right);

				if (state == normal){
					if (front_left < 15 || front_right < 15){
						angle = lastServo;
					}
					else{
						frontLinear = 1/(float)front_left-1/(float)front_right;
						if(frontLinear >= 0.003 || frontLinear <= - 0.003){
							angle = servoPIDCurv.getPID(0.0,frontLinear);
						}else{
							angle = servoPIDStraight.getPID(0.0,frontLinear);
						}
						lastServo = angle;
						if (front_left == front_right){
							equalMin = min(equalMin,front_left);
							equalMax = max(equalMax,front_left);
						}
					}
					led3.SetEnable(0);
					led2.SetEnable(1);
				}
				else if (state == nearLoop){

					if (leftLoop){
						angle = servoPIDOneStraight.getPID((equalMax+equalMin)/2, mid_right);
					}
					else{
						angle = -servoPIDOneStraight.getPID((equalMax+equalMin)/2, mid_left);
					}
					led3.SetEnable(0);
					led2.SetEnable(0);
				}
				else if (state == straight1 || state == straight2 || state == straight3){
					midLinear = 2*(1/(float)mid_left-1/(float)mid_right);
					angle = servoPIDCurve.getPID(0.0,midLinear);
					if (straight1 == state){
						led3.SetEnable(1);
						led2.SetEnable(0);
						led1.SetEnable(1);
					}
					else if (state == straight2){
						led1.SetEnable(0);
					}
					else{
						led1.SetEnable(lastTime % 100 < 50);
					}
				}
				else if (state == turning){
					if (leftLoop){
						if (mid_left < (equalMax+equalMin)/2){
							angle = 0;
						}
						else{
							angle = servoPIDOneCurve.getPID((equalMax+equalMin)/2, mid_left);
							lastServo = angle;
						}
					}
					else{
						if (mid_right < (equalMax+equalMin)/2){
							angle = 0;
						}
						else{
							angle = -servoPIDOneCurve.getPID((equalMax+equalMin)/2, mid_right);
							lastServo = angle;
						}







					}

					lastServo = angle;
					led3.SetEnable(1);
					led2.SetEnable(1);
				}
				else if (state == inLoop){
					if (!bigVal){
						if (front_left < 25 || front_right < 25){
							if (leftLoop){
								angle = 300;
							}
							else{
								angle = -300;
							}
						}
						else{
							frontLinear = 1/(float)front_left-1/(float)front_right;
							angle = servoPIDCurv.getPID(0.0,frontLinear);
							lastServo = angle;
						}
						if (max(front_left, front_right) > oneLineMax){
							bigVal = true;
						}
					}
					else{
						if (front_left < 50 || front_right < 50){
							if (leftLoop){
								angle = 300;
							}
							else{
								angle = -300;
							}
						}
						else{
							frontLinear = 2*(1/(float)front_left-1/(float)front_right);
							angle = servoPIDCurv.getPID(0.0,frontLinear);
						}
						led1.SetEnable(lastTime % 100 < 50);
					}
					led3.SetEnable(lastTime % 100 < 50);
					led2.SetEnable(lastTime % 100 < 50);
				}
				else if (state == outLoop){
					if (front_left < 30 || front_right < 30){
						if (leftLoop){
							angle = 300;
						}
						else{
							angle = -300;
						}
					}
					else{
						frontLinear = 1/(float)front_left-1/(float)front_right;
						angle = servoPIDCurv.getPID(0.0,frontLinear);
						lastServo = angle;
					}
					led3.SetEnable(lastTime % 100 < 50);
					led2.SetEnable(1);
				}
				led0.SetEnable(!leftLoop);

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

				if (start){
					motorL.SetPower(motorLPID.getPID());
					motorR.SetPower(motorRPID.getPID());
				}
				mBT.sendVelocity();
			}

			filterCounter++;
			filterSum0 += mid_left;
			filterSum1 += mid_right;
			filterSum2 += front_left;
			filterSum3 += front_right;

			if (lastTime % 100 == 0){
				if(!start){
					char c[10];
					lcd.SetRegion(Lcd::Rect(0,0,128,15));
					sprintf(c,"FL: %d", front_left);
					writer.WriteBuffer(c,10);
					lcd.SetRegion(Lcd::Rect(0,15,128,15));
					sprintf(c,"FR: %d", front_right);
					writer.WriteBuffer(c,10);
					lcd.SetRegion(Lcd::Rect(0,30,128,15));
					sprintf(c,"ML: %d",mid_left);
					writer.WriteBuffer(c,10);
					lcd.SetRegion(Lcd::Rect(0,45,128,15));
					sprintf(c,"MR: %d",mid_right);
					writer.WriteBuffer(c,10);
					lcd.SetRegion(Lcd::Rect(0,60,128,15));
					sprintf(c,"P: %d",(int)servoPIDCurve.getkP());
					writer.WriteBuffer(c,10);
					lcd.SetRegion(Lcd::Rect(0,75,128,15));
					sprintf(c,"D: %d",(int)servoPIDCurve.getkD());
					writer.WriteBuffer(c,10);
					lcd.SetRegion(Lcd::Rect(0,90,128,15));
					sprintf(c,"A: %d",servo.GetDegree());
					writer.WriteBuffer(c,10);
					lcd.SetRegion(Lcd::Rect(0,105,128,15));
					sprintf(c,"S: %d",(int)speed);
					writer.WriteBuffer(c,10);
					lcd.SetRegion(Lcd::Rect(0,120,128,15));
					sprintf(c,"M: %f",frontLinear*100);
					writer.WriteBuffer(c,10);
//					if (state == normal){
//						lcd.FillColor(0xFF00);
//					}else if (state == nearLoop){
//						if (isLeft){
//							lcd.SetRegion(Lcd::Rect(0,120,64,15));
//						}
//						else{
//							lcd.SetRegion(Lcd::Rect(64,120,64,15));
//						}
//						lcd.FillColor(0x0FF0);
//					}else if (state == turning){
//						lcd.FillColor(0x00FF);
//					}else if (state == inLoop){
//						lcd.FillColor(0x0000);
//					}
					lcd.SetRegion(Lcd::Rect(0,135,128,15));
					sprintf(c,"M: %f",midLinear*100);
//					sprintf(c,"%d %d", equalMin, equalMax);
//					sprintf(c,"%d%d%d%d", minLeft, maxLeft, minRight, maxRight);
					writer.WriteBuffer(c, 10);
				}
			}
    	}
    }
    return 0;
}

