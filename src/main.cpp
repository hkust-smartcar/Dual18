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
    PID servoPID(2500,20000);
    PID motorLPID(0.3,0.0,0.0, &dirEncoder);
    PID motorRPID(0.6,0.0,0.0, &dirEncoder);
    bt mBT(&servoPID, &motorLPID, &motorRPID);

	typedef enum {
		normal = 0,
		nearLoop,
		straight,
		turning,
		inloop
	}carState;
	carState state = normal;
	bool start = false;
    uint32_t lastTime = 0;
    uint32_t greenTime = 0;
	bool dir = 1;
	uint8_t left_mag, right_mag, mid_mag;
	float angle = 0;
	float left_x, right_x;
	const float left_k = 767.2497;
	const float right_k = 854.7614;
	const float h = 6.2;
	float magRatio, xRatio;


    Joystick js(myConfig::GetJoystickConfig(Joystick::Listener([&motor, &start, &led2](const uint8_t id, const Joystick::State state){
    	if(state == Joystick::State::kLeft){
    		int a = 0;
    	}
    	else if (state == Joystick::State::kUp){
    		start = true;
			motor.SetPower(110);
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
			if (lastTime % 10 == 0){
				left_mag = mag0.GetResult();
				right_mag = mag1.GetResult();
				mid_mag = mag2.GetResult();
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

				if((state == normal && mid_mag >= 120 && (left_mag >= 90 || right_mag >= 90))){
					servo.SetDegree(angle);
					lastAngle = angle;
					state = nearLoop;
					if(left_mag >= 85){
						dir = 0;
						greenTime = lastTime;
					}else if(right_mag >= 85){
						dir = 1;
						greenTime = lastTime;
					}
				}
				else if((state == nearLoop &&  mid_mag >= 120 && (!dir?left_mag <= 70:right_mag<= 70))){
					servo.SetDegree(!dir?600:1200);
					state = straight;
				}else if(state == nearLoop){
					servo.SetDegree(!dir?600:1200);
				}else if(state == straight &&  mid_mag >= 120 && (!dir?left_mag >= 80:right_mag >= 80)){
					servo.SetDegree(angle);
					state = turning;
				}
				else if(state == turning){
					if(!dir){
						servo.SetDegree(1700);
					}else{
						servo.SetDegree(100);
					}
					if(mid_mag <= 120 && (dir?(right_mag <= 50 &&left_mag <= 60):(left_mag <= 50 && right_mag <= 60))){
						state = inloop;
					}
				}else if(state != turning && state != straight && state != nearLoop) {
					servo.SetDegree(angle);
				}

//				if (mid_mag > 129){
//					if (dir){
//						servo.SetDegree(300);
//					}
//					else{
//						servo.SetDegree(1500);
//					}
//					state = turning;
//				}
//				else if (state == turning && left_mag+right_mag > 150){
//					if (dir){
//						servo.SetDegree(0);
//					}
//					else{
//						servo.SetDegree(1800);
//					}
//				}
//				else if (left_mag > 100 && left_mag-right_mag > 50){
//					if(lastTime-greenTime > 3000){
//						dir = 0;
//						greenTime = lastTime;
//					}
//					servo.SetDegree(angle);
//					state = nearLoop;
//				}
//				else if (right_mag > 100 && right_mag-left_mag > 50){
//					if(lastTime-greenTime > 3000 ){
//						dir = 1;
//						greenTime = lastTime;
//					}
//					servo.SetDegree(angle);
//					state = nearLoop;
//				}
//				else{
//					servo.SetDegree(angle);
//					state = normal;
//				}

//				if(left_mag+right_mag > 100){
//					magRatio = (float)left_mag/(right_mag+left_mag);
//					if (magRatio < 0.35 && state != transit){
//						dir = 1;
//						servo.SetDegree(angle);
//						state = nearLoop;
//					}
//					else if (magRatio > 0.65 && state != transit){
//						dir = 0;
//						servo.SetDegree(angle);
//						state = nearLoop;
//					}
//					else if (magRatio > 0.40 && magRatio < 0.60){
//						if (dir){
//							servo.SetDegree(0);
//						}
//						else{
//							servo.SetDegree(1800);
//						}
//						state = turning;
//						lcd.SetRegion(Lcd::Rect(0,105,128,15));
//						lcd.FillColor(0x00FF);
//					}
//					else{
//						servo.SetDegree(angle);
//						state = transit;
//					}
//				}
//				else{
//					servo.SetDegree(angle);
//					state = normal;
//				}

//				servo.SetDegree(angle);
			}

			if (lastTime % 100 == 0){
////				if(start){
////					dirEncoder.Update();
//////					motor.SetPower(motorLPID.getPID(0 - dirEncoder.GetCount()));
////				}
////				led3.Switch();
				char c[10];
				lcd.SetRegion(Lcd::Rect(0,0,128,15));
				sprintf(c,"L: %d",left_mag);
				writer.WriteBuffer(c,10);
				lcd.SetRegion(Lcd::Rect(0,15,128,15));
				sprintf(c,"R: %d",right_mag);
				writer.WriteBuffer(c,10);
////				lcd.SetRegion(Lcd::Rect(0,30,128,15));
////				sprintf(c,"X: %f",left_x);
////				writer.WriteBuffer(c,10);
////				lcd.SetRegion(Lcd::Rect(0,45,128,15));
////				sprintf(c,"X: %f",right_x);
////				writer.WriteBuffer(c,10);
////				lcd.SetRegion(Lcd::Rect(0,60,128,15));
////				sprintf(c,"D: %d",dir);
////				writer.WriteBuffer(c,10);
////				lcd.SetRegion(Lcd::Rect(0,75,128,15));
////				sprintf(c,"L: %d",lastTime);
////				writer.WriteBuffer(c,10);
////				lcd.SetRegion(Lcd::Rect(0,90,128,15));
////				sprintf(c,"G: %d",greenTime);
////				writer.WriteBuffer(c,10);
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
					lcd.FillColor(0xFFFF);
				}
				lcd.SetRegion(Lcd::Rect(0,120,128,15));
				sprintf(c,"M: %d",mid_mag);
				writer.WriteBuffer(c,10);
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

