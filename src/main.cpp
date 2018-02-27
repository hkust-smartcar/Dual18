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

    DirEncoder dirEncoder(myConfig::GetEncoderConfig());
    PID servoPID(2450,23000);
    PID motorLPID(0.3,0.0,0.0, &dirEncoder);
    PID motorRPID(0.6,0.0,0.0, &dirEncoder);
    bt mBT(&servoPID, &motorLPID, &motorRPID);

	typedef enum {
		normal = 0,
		verynearLoop,
		nearLoop,
		straight,
		turning,
		turning2,
		inloop
	}carState;
	carState state = normal;
	bool start = false;
    uint32_t lastTime = 0;
    uint32_t greenTime = 0;
	bool dir = 1;
	uint8_t left_mag, right_mag, mid_left, mid_right;
	float angle = 0;
	float left_x, right_x;
	const float left_k = 760.4566;
	const float right_k = 852.0975;
	const float h = 6.2;
	float magRatio, xRatio;
	int power = 150;

    Joystick js(myConfig::GetJoystickConfig(Joystick::Listener([&power,&motor, &start, &led2](const uint8_t id, const Joystick::State state){
    	if (state == Joystick::State::kUp){
    		start = true;
    		motor.SetPower(power);
		}
		else{
			start = false;
			motor.SetPower(0);
		}
    	led2.Switch();
    })));

	servo.SetDegree(900);
	int counter = 0;
	float lastAngle = 0;
    while(1){
    	if(System::Time() != lastTime){
    		lastTime = System::Time();
			if (lastTime % 5 == 0){
				counter ++;
				left_mag = mag0.GetResult();
				right_mag = mag1.GetResult();
				mid_left = mag2.GetResult();
				mid_right = mag3.GetResult();
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
				angle = servoPID.getPID(0.5,xRatio);
				angle += 900;
				if (angle > 1800) {
					angle = 1800;
				}
				else if (angle < 0){
					angle = 0;
				}

				if(state == normal && ((mid_left >= 105 && (left_mag + right_mag) >= 120) || (mid_right >= 105 && (left_mag + right_mag) >= 120))){
					state = verynearLoop;
					power = 90;
					servo.SetDegree(angle);
					if(mid_left >= 105 && left_mag > right_mag){
						dir = 0;
					}else if(mid_right >= 105 && right_mag < left_mag){
						dir = 1;
					}
				}
				else if(state == verynearLoop && (!dir? left_mag >= 100: right_mag >= 100)){
					state = nearLoop;
					servo.SetDegree(angle);
				}
				else if(state == verynearLoop){
					if(!dir){
						if(angle <= 800){
							angle = 800;
						}else if(angle >= 1100){
							angle = 1100;
						}
					}else{
						if(angle <= 800){
							angle = 800;
						}else if(angle >= 1200){
							angle = 1200;
						}
					}
					servo.SetDegree(angle);
				}
				else if((state == nearLoop && (!dir?left_mag <= 80:right_mag <= 80))){
					state = straight;
					servo.SetDegree(angle);
				}else if(state == nearLoop){
					if(!dir){
						if(angle <= 800){
							angle = 800;
						}else if(angle >= 1100){
							angle = 1100;
						}
					}else{
						if(angle <= 800){
							angle = 800;
						}else if(angle >= 1200){
							angle = 1200;
						}
					}
					servo.SetDegree(angle);
				}else if(state == straight && (!dir?left_mag >= 80:right_mag >= 80)){
					state = turning;
					servo.SetDegree(angle);
					power = 90;
				}else if(state == straight){
					if(!dir){
						if(angle <= 800){
							angle = 800;
						}else if(angle >= 1100){
							angle = 1100;
						}
					}else{
						if(angle <= 800){
							angle = 800;
						}else if(angle >= 1200){
							angle = 1200;
						}
					}
					servo.SetDegree(angle);
				}
				else if(state == turning){
					if(!dir){
						servo.SetDegree(1800);
					}else{
						servo.SetDegree(0);
					}
					if(dir?((left_x+right_x)>=21 && (left_mag + right_mag) <= 120) && right_mag > left_mag:(left_x+right_x)>=21 && (left_mag + right_mag) <= 140 && left_mag < right_mag){
						state = inloop;
						power = 130;
					}
				}
				else if(state != turning && state != straight && state != nearLoop) {
					servo.SetDegree(angle);
				}
//				if(start)
//					motor.SetPower(power);
				motor.SetPower(motorLPID.getPID(0));
				mBT.sendVelocity();
			}

			if (lastTime % 100 == 0){
				if(!start){
					char c[10];
					lcd.SetRegion(Lcd::Rect(0,0,128,15));
					sprintf(c,"L: %f",motorLPID.getcurrentVelocity());
					writer.WriteBuffer(c,10);
					lcd.SetRegion(Lcd::Rect(0,15,128,15));
					sprintf(c,"R: %d",right_mag);
					writer.WriteBuffer(c,10);
					lcd.SetRegion(Lcd::Rect(0,30,128,15));
					sprintf(c,"ML: %d",mid_left);
					writer.WriteBuffer(c,10);
					lcd.SetRegion(Lcd::Rect(0,45,128,15));
					sprintf(c,"MR: %d",mid_right);
					writer.WriteBuffer(c,10);
					lcd.SetRegion(Lcd::Rect(0,60,128,15));
					sprintf(c,"c: %d",counter);
					writer.WriteBuffer(c,10);
					lcd.SetRegion(Lcd::Rect(0,105,128,15));
					if (state == normal){
						lcd.FillColor(0xFF00);
					}else if (state == nearLoop){
						if (dir){
							lcd.SetRegion(Lcd::Rect(64,105,64,15));
						}
						else{
							lcd.SetRegion(Lcd::Rect(0,105,64,15));
						}
						lcd.FillColor(0x0FF0);
					}else if (state == turning){
						lcd.FillColor(0x00FF);
					}else if (state == inloop){
						lcd.FillColor(0x0000);
					}
				}
			}

//    		if(lastTime % 600 == 0){
//    			char c[10];
//    			lcd.SetRegion(Lcd::Rect(0,0,128,15));
//    			sprintf(c,"servoP: %f", servoPID.getkP());
//    			writer.WriteBuffer(c,15);
//    			lcd.SetRegion(Lcd::Rect(0,15,128,15));
//    			sprintf(c,"servoD: %f", servoPID.getkD());
//    			writer.WriteBuffer(c,15);
//    			lcd.SetRegion(Lcd::Rect(0,30,128,15));
//    			dirEncoder.Update();
//    			sprintf(c,"%dLMotorT: %f", dirEncoder.GetCount(),motorLPID.getDesiredVelocty());
//    			writer.WriteBuffer(c,15);
//    			lcd.SetRegion(Lcd::Rect(0,45,128,15));
//    			sprintf(c,"LMotorP: %f", motorLPID.getkP());
//    			writer.WriteBuffer(c,15);
//    			lcd.SetRegion(Lcd::Rect(0,60,128,15));
//    			sprintf(c,"LMotorI: %f", motorLPID.getkI());
//    			writer.WriteBuffer(c,15);
//    			lcd.SetRegion(Lcd::Rect(0,75,128,15));
//    			sprintf(c,"LMotorD: %f", motorLPID.getkD());
//    			writer.WriteBuffer(c,15);
//    			lcd.SetRegion(Lcd::Rect(0,90,128,15));
//    			sprintf(c,"RMotorT: %f", motorRPID.getDesiredVelocty());
//    			writer.WriteBuffer(c,15);
//    			lcd.SetRegion(Lcd::Rect(0,105,128,15));
//    			sprintf(c,"RMotorP: %f", motorRPID.getkP());
//    			writer.WriteBuffer(c,15);
//    			lcd.SetRegion(Lcd::Rect(0,120,128,15));
//    			sprintf(c,"RMotorI: %f", motorRPID.getkI());
//    			writer.WriteBuffer(c,15);
//    			lcd.SetRegion(Lcd::Rect(0,135,128,15));
//    			sprintf(c,"%dRMotorD: %f",mBT.buffer.size(), motorRPID.getkD());
//    			writer.WriteBuffer(c,15);
//    		}
    	}
    }
    return 0;
}

