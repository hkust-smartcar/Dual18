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
#include "libsc/battery_meter.h"
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

float _sqrt(float x){return x*inv_sqrt(x);}

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

	FutabaS3010 servo(myConfig::GetServoConfig());
	AlternateMotor motor(myConfig::GetMotorConfig());
    St7735r lcd(myConfig::GetLcdConfig());
    LcdTypewriter writer(myConfig::GetWriterConfig(&lcd));
    LcdConsole console(myConfig::GetConsoleConfig(&lcd));
    lcd.SetRegion(Lcd::Rect(0,0,128,160));
    lcd.FillColor(lcd.kWhite);
    BatteryMeter battery_meter(myConfig::GetBatteryMeterConfig());

    DirEncoder dirEncoder(myConfig::GetEncoderConfig());
    PID servoPID(2450,23000);
    PID motorLPID(0.3,0.0,0.0, &dirEncoder);
    PID motorRPID(0.6,0.0,0.0, &dirEncoder);
    bt mBT(&servoPID, &motorLPID, &motorRPID);

	typedef enum {
		normal = 0,
		nearLoop,
		turning,
		inside,
		exit1,
		exit2
	}carState;
	carState state = normal;
	bool start = false;
    uint32_t lastTime = 0;
	uint8_t left_mag, right_mag, mid_left, mid_right;
//	uint32_t left_sum = 0, right_sum = 0, cycleCount = 0;
	float angle = 0;
	float left_x, right_x;
	const float left_k = 760.4566;
	const float right_k = 852.0975;
	const float h = 6.2;
	float magRatio, xRatio;
	bool left = 0;
	uint8_t count = 0;
	bool waitTrigger = 1, detectLoop = 0;
	uint8_t speed = 120;
	int8_t encoderSign = -1;

    Joystick js(myConfig::GetJoystickConfig(Joystick::Listener([&motor, &start, &led2, &speed](const uint8_t id, const Joystick::State state){
    	if (state == Joystick::State::kSelect){
    		start = true;
    		speed += 10;
			motor.SetPower(speed);
		}
		else{
			start = false;
			speed = 120;
			motor.SetPower(0);
		}
    	led2.Switch();
    })));

	servo.SetDegree(900);
	uint16_t servo_mid = 900;

	float battery = battery_meter.GetVoltage();
	lcd.SetRegion(Lcd::Rect(0,0,128,160));
	if (battery > 8){
		lcd.FillColor(0xF100);
	}
	else if (battery > 7.7){
		lcd.FillColor(0x0EE0);
	}
	else if (battery > 7.5){
		lcd.FillColor(0x001F);
	}
	else{
		lcd.FillColor(0xFFFF);
	}

    while(1){
    	if(System::Time() != lastTime){
    		lastTime = System::Time();
//    		if(lastTime % 100 == 0){
//				mBT.sendVelocity();
//    		}

			if (lastTime % 5 == 0){
				left_mag = mag0.GetResult();
				right_mag = mag1.GetResult();
				mid_left = mag2.GetResult();
				mid_right = mag3.GetResult();
//				cycleCount++;
//				left_sum += left_mag;
//				right_sum += right_mag;
				if (left_k*h/left_mag < h*h){
					left_x = 0;
				}
				else{
					left_x = _sqrt(left_k*h/left_mag-h*h);
				}
				if (right_k*h/right_mag < h*h){
					right_x = 0;
				}
				else{
					right_x = _sqrt(right_k*h/right_mag-h*h);
				}
				xRatio = (float)left_x/(right_x+left_x);

				if (count == 0 && ((mid_left>100 && mid_right>40 && left_mag>right_mag) || (mid_right>100 && mid_left>40 && right_mag>left_mag)) && mid_left+mid_right>150){
					detectLoop = 1;
					if (mid_left>100 && mid_right>40 && left_mag>right_mag){
						left = 1;
					}
					else{
						left = 0;
					}
					state = nearLoop;
				}
				if (detectLoop && state != turning && waitTrigger && left_x+right_x <= 17 && (left_mag > 90 || right_mag > 90)){ //sum of x may not be needed
					count++;
					waitTrigger = 0;
					if (count == 2){
						state = turning;
					}
					else if (count > 2 && state == inside){
						state = exit1;
						lcd.SetRegion(Lcd::Rect(0,0,128,10));
						lcd.FillColor(0x0EE0);
					}
				}
				if (!waitTrigger && left_mag <= 75 && right_mag <= 75 && left_mag+right_mag <= 135){
					waitTrigger = 1;
				}
				if (!waitTrigger && state == exit1 && left_mag <= 90 && right_mag <= 90 && 0.3 < xRatio && xRatio < 0.7 && left_x+right_x > 10){
					state = exit2;
					lcd.SetRegion(Lcd::Rect(0,0,128,10));
					lcd.FillColor(0x001F);
				}
				if(state == exit2 && left_x+right_x > 20){
					detectLoop = 0;
					state = normal;
					count = 0;
					lcd.SetRegion(Lcd::Rect(0,0,128,10));
					lcd.FillColor(0xFFFF);
				}

				angle = servoPID.getPID(0.5,xRatio);
				angle += servo_mid;
				angle = min(max(angle,0),1800);
				if (state == turning){
					if (left_x+right_x > 20){
						state = inside;
					}
					if (left){
						servo.SetDegree(angle<servo_mid+350?servo_mid+600:angle);
					}
					else{
						servo.SetDegree(angle>servo_mid-350?servo_mid-600:angle);
					}
				}
				if (state == normal || state == inside){
					servo.SetDegree(angle);
				}
				else if (state == nearLoop){
					if (left){
						servo.SetDegree(angle>servo_mid+150?servo_mid+150:angle);
					}
					else{
						servo.SetDegree(angle<servo_mid-150?servo_mid-150:angle);
					}
				}
				else if (state == exit1){
					if (!left){
						servo.SetDegree(angle>servo_mid+450?servo_mid+450:angle);
					}
					else{
						servo.SetDegree(angle<servo_mid-450?servo_mid-450:angle);
					}
				}
				else if (state == exit2){
					if (left){
						servo.SetDegree(angle>servo_mid+100?servo_mid+100:angle);
					}
					else{
						servo.SetDegree(angle<servo_mid-100?servo_mid-100:angle);
					}
				}

//				motor.SetPower(motorLPID.getPID(0));
//				if (start){
//				}
			}

			if (lastTime % 100 == 0){
				if(start){
					dirEncoder.Update();
					int32_t temp = -dirEncoder.GetCount();
					if (temp>530){
						speed -= 3;
					}
					else if (temp<510){
						speed += 3;
					}
					speed = min(170,max(speed,90));
					motor.SetPower(speed);
//					char c[10];
//					lcd.SetRegion(Lcd::Rect(0,0,128,15));
//					sprintf(c,"M: %d",motor.GetPower());
//					writer.WriteBuffer(c,10);
//					lcd.SetRegion(Lcd::Rect(0,15,128,15));
//					sprintf(c,"E: %d",temp);
//					writer.WriteBuffer(c,10);
				}
				else{
					led3.Switch();
					char c[10];
					lcd.SetRegion(Lcd::Rect(0,0,128,15));
					sprintf(c,"R: %d",left_mag);
					writer.WriteBuffer(c,10);
					lcd.SetRegion(Lcd::Rect(0,15,128,15));
					sprintf(c,"R: %d",right_mag);
					writer.WriteBuffer(c,10);
					lcd.SetRegion(Lcd::Rect(0,30,128,15));
					sprintf(c,"M: %d",mid_left);
					writer.WriteBuffer(c,10);
					lcd.SetRegion(Lcd::Rect(0,45,128,15));
					sprintf(c,"M: %d",mid_right);
					writer.WriteBuffer(c,10);
					lcd.SetRegion(Lcd::Rect(0,60,128,15));
					sprintf(c,"X: %f",left_x);
					writer.WriteBuffer(c,10);
					lcd.SetRegion(Lcd::Rect(0,75,128,15));
					sprintf(c,"X: %f",right_x);
					writer.WriteBuffer(c,10);
					lcd.SetRegion(Lcd::Rect(0,90,128,15));
					sprintf(c,"S: %d",left_mag+right_mag);
					writer.WriteBuffer(c,10);
					lcd.SetRegion(Lcd::Rect(0,105,128,15));
					if (state == normal){
						lcd.FillColor(0xFF00);
					}else if (state == exit1){
						lcd.FillColor(0xF100);
					}else if (state == exit2){
						lcd.FillColor(0x001F);
					}else if (state == nearLoop){
						if (!left){
							lcd.SetRegion(Lcd::Rect(64,105,64,15));
						}
						else{
							lcd.SetRegion(Lcd::Rect(0,105,64,15));
						}
						lcd.FillColor(0x0FF0);
					}else if (state == turning){
						lcd.FillColor(0x00FF);
					}else if (state == inside){
						lcd.FillColor(0x0000);
					}
					lcd.SetRegion(Lcd::Rect(0,120,128,15));
					sprintf(c,"R: %f",xRatio);
					writer.WriteBuffer(c,10);
					lcd.SetRegion(Lcd::Rect(0,135,128,15));
					sprintf(c,"S: %f",left_x+right_x);
					writer.WriteBuffer(c,10);
				}
			}
    	}
    }
    return 0;
}

