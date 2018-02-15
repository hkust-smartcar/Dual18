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

    DirEncoder dirEncoder(myConfig::GetEncoderConfig());
    PID servoPID(3000,0);
    PID motorLPID(1.0,0.0,0.0, &dirEncoder);
    PID motorRPID(0.0,0.0,0.0, &dirEncoder);
    bt mBT(&servoPID, &motorLPID, &motorRPID);

    bool start = false;
    Joystick js(myConfig::GetJoystickConfig(Joystick::Listener([&motor, &start](const uint8_t id, const Joystick::State state){
    	if(state == Joystick::State::kLeft){
    		int a = 0;
    	}
    	else if (state == Joystick::State::kUp){
    		start = true;
		}
		else if (state == Joystick::State::kDown){
			start = false;
		}
    })));

	servo.SetDegree(900);

    while(1){
    	if(System::Time() != lastTime){
    		lastTime = System::Time();
			if (lastTime % 40 == 0){
				led3.Switch();
				motor.SetPower(motorLPID.getPID());
				mBT.sendVelocity();
				char c[10];
				lcd.SetRegion(Lcd::Rect(0,0,128,15));
				sprintf(c,"Motor: %d",motor.GetPower());
				writer.WriteBuffer(c,10);
			}
    	}
    }
    return 0;
}

