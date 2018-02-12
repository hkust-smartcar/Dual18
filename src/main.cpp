/*
 * main.cpp
 *
 * Author: 
 * Copyright (c) 2014-2015 HKUST SmartCar Team
 * Refer to LICENSE for details
 */

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

    uint32_t lastTime = 0;
	int count = 0;
	uint32_t left_sum = 0, right_sum = 0;

    DirEncoder dirEncoder(myConfig::GetEncoderConfig());
    PID servoPID(3000,0);
    PID motorLPID(0.3,0.0,0.0, &dirEncoder);
    PID motorRPID(0.6,0.0,0.0, &dirEncoder);
    bt mBT(&servoPID, &motorLPID, &motorRPID);

    bool start = false;
    Joystick js(myConfig::GetJoystickConfig(Joystick::Listener([&motor, &start](const uint8_t id, const Joystick::State state){
    	if(state == Joystick::State::kLeft){
    		int a = 0;
    	}
    	else if (state == Joystick::State::kUp){
    		start = true;
			motor.SetPower(110);
		}
		else if (state == Joystick::State::kDown){
			start = false;
			motor.SetPower(0);
		}
    })));

	servo.SetDegree(900);

    while(1){
    	if(System::Time() != lastTime){
    		lastTime = System::Time();
    		if(lastTime % 100 == 0){
				mBT.sendVelocity();
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

			uint16_t left_mag, right_mag;
			float angle = 0;
			float left_x, right_x;
			const float left_k = 767.2497;
			const float right_k = 854.7614;
			const float h = 6.2;
			if (lastTime % 10 == 0){

				led3.Switch();

				count++;
				left_mag = mag0.GetResult();
				right_mag = mag1.GetResult();
				left_sum += left_mag;
				right_sum += right_mag;
				left_x = _sqrt(left_k*h/left_mag-h*h);
				right_x = _sqrt(right_k*h/right_mag-h*h);

				float ratio = left_x/(right_x+left_x);
//				double error, derivative, output;
//				error = 0.5-ratio;
//				integral = integral + error*dt;
//				derivative = (error-previous_error)/dt;
//				output = kp*error + ki*integral + kd*derivative;
//				previous_error = error;
//				angle = 900+output;
				angle = servoPID.getPID(0.5,(float)ratio);
				angle += 900;
//				Angle = angle;
				if (angle > 1800) {
					angle = 1800;
				}
				else if (angle < 0){
					angle = 0;
				}
				servo.SetDegree((uint16_t)(angle));
			}

			if (lastTime % 100 == 0){
				char c[10];
				lcd.SetRegion(Lcd::Rect(0,0,128,15));
				sprintf(c,"R: %d",left_mag);
				writer.WriteBuffer(c,10);
				lcd.SetRegion(Lcd::Rect(0,15,128,15));
				sprintf(c,"R: %d",right_mag);
				writer.WriteBuffer(c,10);
				lcd.SetRegion(Lcd::Rect(0,30,128,15));
				sprintf(c,"X: %f",left_x);
				writer.WriteBuffer(c,10);
				lcd.SetRegion(Lcd::Rect(0,45,128,15));
				sprintf(c,"X: %f",right_x);
				writer.WriteBuffer(c,10);\
				lcd.SetRegion(Lcd::Rect(0,60,128,15));
				sprintf(c,"R: %f",left_x/(right_x+left_x));
				writer.WriteBuffer(c,10);
				lcd.SetRegion(Lcd::Rect(0,75,128,15));
				sprintf(c,"A: %f",angle);
				writer.WriteBuffer(c,10);
//				lcd.SetRegion(Lcd::Rect(0,90,128,15));
//				sprintf(c,"O: %f",output);
//				writer.WriteBuffer(c,10);
//				lcd.SetRegion(Lcd::Rect(0,90,128,15));
//				sprintf(c,"I: %f",integral);
//				writer.WriteBuffer(c,10);
				if(start){
					dirEncoder.Update();
					motor.SetPower(motorLPID.getPID(0 - dirEncoder.GetCount()));
				}
			}
    	}
    }
    return 0;
}

