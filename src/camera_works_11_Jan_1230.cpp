/*
 * main.cpp
 *
 * Author:
 * Copyright (c) 2014-2015 HKUST SmartCar Team
 * Refer to LICENSE for details
 */

#include <cassert>
#include <cstring>
#include <libbase/k60/mcg.h>
#include <libsc/system.h>
#include <libsc/led.h>
#include <libsc/lcd_typewriter.h>
#include <libbase/k60/gpio.h>
#include <libsc/st7735r.h>
#include <libsc/lcd.h>
#include <libsc/k60/ov7725.h>
#include <libsc/k60/ov7725_configurator.h>
#include <libsc/lcd_typewriter.h>
#include <libsc/futaba_s3010.h>
#include <vector>
#include <math.h>
#include <libsc/alternate_motor.h>
#include "coord.h"

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
using namespace std;

#define Width 80
#define Height 60
bool camptr[Height][Width];

void extract_cam (const Byte* camBuffer) {
    Uint pos = 0;
    int bit_pos = 8;
    for(Uint i=0; i < Height; i++){
        for(Uint j = 0;j<Width; j++){
            if (--bit_pos < 0) // Update after 8 bits are read
            {
                bit_pos = 7;// to track which position in a branch of Byte(Totally 8) is being read now.
                ++pos;// to track which position in Byte array is being read now.
            }
            camptr[i][j] = GET_BIT(camBuffer[pos], bit_pos);
        }
    }
}

St7735r* lcdP;

St7735r::Config getLcdConfig()
{
    St7735r::Config config;
    config.orientation = 2;
    return config;
}

k60::Ov7725::Config getCameraConfig()
{
    k60::Ov7725::Config config;
    config.id = 0;
    config.w = Width;
    config.h = Height;
    config.fps = k60::Ov7725Configurator::Config::Fps::kHigh;
    return config;
}

FutabaS3010::Config getServoConfig()
{
    FutabaS3010::Config config;
    config.id = 0;
    return config;
}

AlternateMotor::Config getMotorConfig()
{
    AlternateMotor::Config config;
    config.id = 0;
    return config;
}
/*
void Print2D(){
	for (int y=0; y<Height; y++){
		for (int x=0; x<Width; x++){
			lcdP->SetRegion(Lcd::Rect(x, y, 1, 1));
			if (!camptr[y][x]){
				lcdP->FillColor(0xFFFF);
			} else {
				lcdP->FillColor(0x0000);
			}
		}
	}

}
*/
int main(void)
{
    System::Init();

    k60::Ov7725 camera(getCameraConfig());
    camera.Start();

    St7735r lcd(getLcdConfig());
    lcdP = &lcd;

    FutabaS3010 Servo(getServoConfig());

    AlternateMotor Motor(getMotorConfig());

    Motor.SetPower(100);
    Motor.SetClockwise(false);

    Servo.SetDegree(90*10);

    int32_t ticks = System::Time();
    while (true){
        while(ticks!=libsc::System::Time()){
            ticks=libsc::System::Time();
            if(ticks%10==0){
                ticks = System::Time();
                const Byte* camBuffer = camera.LockBuffer();
                extract_cam(camBuffer);
//				Print2D();
                lcdP->SetRegion(Lcd::Rect(0, 0, Width, Height));
                lcdP->FillBits(0x0000, 0xFFFF, camBuffer, Width * Height);
                camera.UnlockBuffer();
                int middle_coor = 40;
                int right_coor = 0;
                int left_coor = 0;
                for(int i=50;i<55;i++){
                    for(int j=40;j<80;j++){
                        if(camptr[i][j] == 1) {right_coor += j;break;}
                        else if (j == 79) right_coor+=j;
                    }
                }
                right_coor /= 5;
                for(int i=50;i<55;i++){
                    for(int j=40;j>0;j--)
                        if(camptr[i][j] == 1) {left_coor += j;break;}
                }
                left_coor /= 5;
                int temp_coor = (left_coor + right_coor)/2;
                Servo.SetDegree((90+temp_coor - 40)*10);
            }
        }
    }

    return 0;
}
