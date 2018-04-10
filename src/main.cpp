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
    PID servoPIDCurve(13000, 0);
    PID servoPIDLeaveCurve(400, 14000);
    PID servoPIDStraight(300,12000);
    PID servoPIDFuzzy((servoPIDCurve.getkP()+servoPIDStraight.getkP())/2,(servoPIDCurve.getkD()+servoPIDStraight.getkD())/2);
//    PID servoPIDCurve(500, 14000);
//    PID servoPIDStraight(250,12000);
    PID motorLPID(0.32,0.0,8, &LEncoder);
    PID motorRPID(0.32,0.0,8, &REncoder);
    float speed = 25;
	float magRatio, xRatio, xRatio2, xRatio3, maxDiff = 0;
    bt mBT(&servoPIDCurve, &servoPIDStraight, &motorLPID, &motorRPID, &speed, &xRatio);
    bool leaveLoop = false;

	typedef enum {
		normal = 0,
		nearLoop,
		straight,
		turning,
		inLoop
	}carState;
	carState state = normal;
	bool start = false;
	float lastServo, frontLinear, midLinear;
	uint16_t filterSum0 = 0, filterSum1 = 0, filterSum2 = 0, filterSum3 = 0, filterSum4 = 0, filterSum5 = 0;
	uint8_t filterCounter = 0;
	uint32_t lastTime = 0;
    uint32_t greenTime = 0;
	bool isLeft = 1;
	bool isCurve = 0;
	uint8_t front_left, front_right, mid_left, mid_right, back_left, back_right;

	int magSum = 0;
	uint16_t middleServo = 865;
	uint16_t leftServo = 1180;
	uint16_t rightServo = 550;
	float angle = middleServo;
	bool tuneP = false;
	int kP = servoPIDCurve.getkP(), kD = servoPIDCurve.getkD();

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
		else if (state == Joystick::State::kUp){
			if (tuneP){
				kP += 1000;
				servoPIDCurve.setkP(kP);
			}
			else{
				kD += 1000;
				servoPIDCurve.setkD(kD);
			}
		}
		else if (state == Joystick::State::kDown){
			if (tuneP){
				kP -= 1000;
				servoPIDCurve.setkP(kP);
			}
			else{
				kD -= 1000;
				servoPIDCurve.setkD(kD);
			}
		}
		else if (state == Joystick::State::kLeft){
			speed += 1;
		}
		else if (state == Joystick::State::kRight){
			tuneP = !tuneP;
		}
    	led2.Switch();
    })));

    float mkTemp;
	servo.SetDegree(middleServo);
    while(1){
    	if(System::Time() != lastTime){
    		lastTime = System::Time();
		if (lastTime % 6 == 0){
				mid_left = filterSum0/filterCounter;
				mid_right = filterSum1/filterCounter;
				front_left = filterSum2/filterCounter;
				front_right = filterSum3/filterCounter;
				back_left = filterSum4/filterCounter;
				back_right = filterSum5/filterCounter;
				filterSum0 = 0;
				filterSum1 = 0;
				filterSum2 = 0;
				filterSum3 = 0;
				filterSum4 = 0;
				filterSum5 = 0;
				filterCounter = 0;

				if (front_left < 15 || front_right < 15){
					angle = lastServo;
				}
				else{
					frontLinear = 1/(float)front_left-1/(float)front_right;
	//				midLinear = 1/(float)mid_left-1/(float)mid_right;
	//				mkTemp = 1/(float)mid_left-1/(float)mid_right;
					angle = servoPIDCurve.getPID(0.0,frontLinear);
					lastServo = angle;
				}

//				if (mid_left < 30 && mid_left < mid_right){
//					mid_right = max(((30-mid_left)/(float)15+1)*mid_right, right_mag*2);
//				}
//				if (mid_right < 30 && mid_right < mid_left){
//					mid_left = max(((30-mid_right)/(float)15+1)*mid_left, left_mag*2);
//				}
//
//
//				magSum = left_mag + right_mag;
//				xRatio = (float)(right_mag - left_mag)/(magSum);
//				xRatio2 = (float)(mid_right - mid_left)/(mid_right + mid_left);
//				xRatio3 = (float)(back_right - back_left)/(back_right + back_left);
//
//				if(xRatio2-xRatio > 0.25 || xRatio2-xRatio < -0.25 || left_mag < 30 || right_mag < 30){
//					angle = servoPIDCurve.getPID(0.0,(xRatio+xRatio2)/2);
////					if (!leaveLoop){
////						angle = servoPIDCurve.getPID(0.0,(xRatio+xRatio2)/2);
////						if (fabs(xRatio2-xRatio) < maxDiff-0.1){
////							leaveLoop = true;
////							maxDiff = 0;
////						}
////						if (xRatio2-xRatio > 0.25 && maxDiff < xRatio2-xRatio){
////							maxDiff = xRatio2-xRatio;
////						}
////						else if (xRatio-xRatio2 > 0.25 && maxDiff < xRatio-xRatio2){
////							maxDiff = xRatio-xRatio2;
////						}
////						led2.SetEnable(1);
////						led3.SetEnable(0);//on
////					}
////					else{
////						angle = servoPIDCurve.getPID(0.0,(2*xRatio+xRatio2)/3);
////						led2.SetEnable(0);
////						led3.SetEnable(1);
////					}
//				}else if(xRatio2-xRatio > 0.15 || xRatio2-xRatio < -0.15){
////					if(xRatio2-xRatio > 0.2){
////						servoPIDFuzzy.setkP((0.25-(xRatio2-xRatio))/0.05*servoPIDStraight.getkP()+((xRatio2-xRatio)-0.2)/0.05*servoPIDCurve.getkP());
////						servoPIDFuzzy.setkD((0.25-(xRatio2-xRatio))/0.05*servoPIDStraight.getkD()+((xRatio2-xRatio)-0.2)/0.05*servoPIDCurve.getkD());
////					}
////					else{
////						servoPIDFuzzy.setkP((0.25-(xRatio-xRatio2))/0.05*servoPIDStraight.getkP()+((xRatio-xRatio2)-0.2)/0.05*servoPIDCurve.getkP());
////						servoPIDFuzzy.setkD((0.25-(xRatio-xRatio2))/0.05*servoPIDStraight.getkD()+((xRatio-xRatio2)-0.2)/0.05*servoPIDCurve.getkD());
////					}
//					angle = servoPIDFuzzy.getPID(0.0,(2*xRatio+xRatio2)/3);
////					led2.SetEnable(0);
////					led3.SetEnable(0);
//				}else{
//					angle = servoPIDStraight.getPID(0.0,xRatio);
////					if (leaveLoop){leaveLoop = false;}
////					led2.SetEnable(1);
////					led3.SetEnable(1);
//				}

				angle += middleServo;
				angle = max(rightServo,min(leftServo,angle));

////				if(state == normal && magSum > 230){
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

				if (1){
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

			filterCounter++;
			filterSum0 += mag0.GetResult();
			filterSum1 += mag1.GetResult();
			filterSum2 += mag2.GetResult();
			filterSum3 += mag3.GetResult();
			filterSum4 += mag4.GetResult();
			filterSum5 += mag5.GetResult();

			if (lastTime % 100 == 0){
				if(!start){
					char c[10];
					lcd.SetRegion(Lcd::Rect(0,0,128,15));
					sprintf(c,"FL: %d", front_left);
					writer.WriteBuffer(c,10);
					lcd.SetRegion(Lcd::Rect(0,15,128,15));
					sprintf(c,"FR: %d", front_right);
					writer.WriteBuffer(c,10);
//					lcd.SetRegion(Lcd::Rect(0,30,128,15));
//					sprintf(c,"ML: %d",mid_left);
//					writer.WriteBuffer(c,10);
//					lcd.SetRegion(Lcd::Rect(0,45,128,15));
//					sprintf(c,"MR: %d",mid_right);
//					writer.WriteBuffer(c,10);
					lcd.SetRegion(Lcd::Rect(0,60,128,15));
					sprintf(c,"P: %d",(int)servoPIDCurve.getkP());
					writer.WriteBuffer(c,10);
					lcd.SetRegion(Lcd::Rect(0,75,128,15));
					sprintf(c,"D: %d",(int)servoPIDCurve.getkD());
//					sprintf(c,"d: %f",servoPID.getdTerm());
					writer.WriteBuffer(c,10);
					lcd.SetRegion(Lcd::Rect(0,90,128,15));
					sprintf(c,"A: %d",servo.GetDegree());
//					sprintf(c,"L: %f",motorLPID.getcurrentVelocity());
					writer.WriteBuffer(c,10);
					lcd.SetRegion(Lcd::Rect(0,105,128,15));
//					sprintf(c,"R: %f",motorRPID.getcurrentVelocity());
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
//					lcd.SetRegion(Lcd::Rect(0,135,128,15));
//					sprintf(c,"xR: %f", xRatio - xRatio2);
//					writer.WriteBuffer(c, 10);
				}
			}
    	}
    }
    return 0;
}

