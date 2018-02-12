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

    St7735r lcd(myConfig::GetLcdConfig());
    LcdTypewriter writer(myConfig::GetWriterConfig(&lcd));
    LcdConsole console(myConfig::GetConsoleConfig(&lcd));
    lcd.SetRegion(Lcd::Rect(0,0,128,160));
    lcd.FillColor(lcd.kWhite);

    uint32_t lastTime = 0;

    DirEncoder dirEncoder(myConfig::GetEncoderConfig());
    PID servoPID(0.1,0.2);
    PID motorLPID(0.3,0.4,0.5, &dirEncoder);
    PID motorRPID(0.6,0.7,0.8, &dirEncoder);
    bt mBT(&servoPID, &motorLPID, &motorRPID);

    Joystick js(myConfig::GetJoystickConfig(Joystick::Listener([](const uint8_t id, const Joystick::State state){
    	if(state == Joystick::State::kLeft){
    		int a = 0;
    	}
    })));


    while(1){
    	if(System::Time() != lastTime){
    		lastTime = System::Time();
    		if(lastTime % 20 == 0){
    			dirEncoder.Update();
    			motorLPID.getPID(dirEncoder.GetCount());
				mBT.sendVelocity();
    		}
    		if(lastTime % 600 == 0){
    			char c[10];
    			lcd.SetRegion(Lcd::Rect(0,0,128,15));
    			sprintf(c,"servoP: %f", servoPID.getkP());
    			writer.WriteBuffer(c,15);
    			lcd.SetRegion(Lcd::Rect(0,15,128,15));
    			sprintf(c,"servoD: %f", servoPID.getkD());
    			writer.WriteBuffer(c,15);
    			lcd.SetRegion(Lcd::Rect(0,30,128,15));
    			dirEncoder.Update();
    			sprintf(c,"%dLMotorT: %f", dirEncoder.GetCount(),motorLPID.getDesiredVelocty());
    			writer.WriteBuffer(c,15);
    			lcd.SetRegion(Lcd::Rect(0,45,128,15));
    			sprintf(c,"LMotorP: %f", motorLPID.getkP());
    			writer.WriteBuffer(c,15);
    			lcd.SetRegion(Lcd::Rect(0,60,128,15));
    			sprintf(c,"LMotorI: %f", motorLPID.getkI());
    			writer.WriteBuffer(c,15);
    			lcd.SetRegion(Lcd::Rect(0,75,128,15));
    			sprintf(c,"LMotorD: %f", motorLPID.getkD());
    			writer.WriteBuffer(c,15);
    			lcd.SetRegion(Lcd::Rect(0,90,128,15));
    			sprintf(c,"RMotorT: %f", motorRPID.getDesiredVelocty());
    			writer.WriteBuffer(c,15);
    			lcd.SetRegion(Lcd::Rect(0,105,128,15));
    			sprintf(c,"RMotorP: %f", motorRPID.getkP());
    			writer.WriteBuffer(c,15);
    			lcd.SetRegion(Lcd::Rect(0,120,128,15));
    			sprintf(c,"RMotorI: %f", motorRPID.getkI());
    			writer.WriteBuffer(c,15);
    			lcd.SetRegion(Lcd::Rect(0,135,128,15));
    			sprintf(c,"%dRMotorD: %f",mBT.buffer.size(), motorRPID.getkD());
    			writer.WriteBuffer(c,15);
    		}
    	}
    }
    return 0;
}

