/*
 * main.cpp
 *
 * Author: Amrutavarsh S Kinagi
 * Copyright (c) 2014-2015 HKUST SmartCar Team
 * Refer to LICENSE for details
 */

#include <cassert>
#include <cstring>
#include <functional>
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
#include <libbase/k60/adc.h>
#include <libsc/joystick.h>
#include <libsc/dir_encoder.h>
#include "coord.h"

namespace libbase {
namespace k60 {

Mcg::Config Mcg::GetMcgConfig() {
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

inline bool ret_cam_bit(int x, int y, const Byte* camBuffer) {
	return ((camBuffer[y * 10 + x / 8] >> (7 - (x % 8))) & 1);
}
inline void set_cam_bit(int x, int y, const Byte* camBuffer) {
	camBuffer[y * 10 + x / 8] >> (7 - (x % 8)) = 1;
}
//
//void extract_cam (const Byte* camBuffer) {
//    Uint pos = 0;
//    int bit_pos = 8;
//    for(Uint i=0; i < Height; i++){
//        for(Uint j = 0;j<Width; j++){
//            if (--bit_pos < 0) // Update after 8 bits are read
//            {
//                bit_pos = 7;// to track which position in a branch of Byte(Totally 8) is being read now.
//                ++pos;// to track which position in Byte array is being read now.
//            }
//            camptr[i][j] = GET_BIT(camBuffer[pos], bit_pos);
//        }
//    }
//}

St7735r* lcdP;

St7735r::Config getLcdConfig() {
	St7735r::Config config;
	config.orientation = 2;
	return config;
}

LcdTypewriter::Config GetWriterConfig(St7735r *temp_lcd) {
	LcdTypewriter::Config config;
	config.lcd = temp_lcd;
	config.bg_color = 0;
	config.text_color = 0xFFFF;
	config.is_text_wrap = false;
	return config;
}

k60::Ov7725::Config getCameraConfig() {
	k60::Ov7725::Config config;
	config.id = 0;
	config.w = Width;
	config.h = Height;
	config.fps = k60::Ov7725Configurator::Config::Fps::kHigh;
	return config;
}

FutabaS3010::Config getServoConfig() {
	FutabaS3010::Config config;
	config.id = 0;
	return config;
}

AlternateMotor::Config getMotorConfig() {
	AlternateMotor::Config config;
	config.id = 0;
	return config;
}

Adc::Config getMagSensorConfig(Pin::Name pin_name) {
	Adc::Config config;
	config.pin = pin_name;
	config.speed = Adc::Config::SpeedMode::kFast;
	config.is_continuous_mode = true;
	config.avg_pass = Adc::Config::AveragePass::k32;
	return config;
}

Joystick::Config getJoystickConfig(Joystick::Listener isr) {
	Joystick::Config config;
	config.id = 0;
	config.is_active_low = true;
	config.dispatcher = isr;
	return config;
}

Encoder::Config getEncoderConfig(){
	Encoder::Config config;
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
int main(void) {
	System::Init();

	k60::Ov7725 camera(getCameraConfig());
	camera.Start();

	St7735r lcd(getLcdConfig());
	lcdP = &lcd;

	LcdTypewriter writer(GetWriterConfig(&lcd));

	FutabaS3010 Servo(getServoConfig());

	AlternateMotor Motor(getMotorConfig());
	Motor.SetPower(150);
	Motor.SetClockwise(false);

//	Adc MagSensor1(getMagSensorConfig(Pin::Name::kPtb0));
//	Adc MagSensor2(getMagSensorConfig(Pin::Name::kPtb1));
//	Adc MagSensor3(getMagSensorConfig(Pin::Name::kPtc10));
//	Adc MagSensor4(getMagSensorConfig(Pin::Name::kPtc11));

	int32_t ticks = System::Time();
//    int32_t brake_ticks = System::Time();
	int pre_temp = 0;
	int temp = 0;
	float derivative;
	int Kp = 30;
	int Kd = 40;
//	bool Motor_start = true;

//	Joystick::Config t = getJoystickConfig(
//			[ ](const uint8_t id, const Joystick::State state) {
//				if(state == Joystick::State::kRight)
//				Kp += 0.02;
//				else if(state == Joystick::State::kLeft)
//				Kp -= 0.02;
//				else if(state == Joystick::State::kUp)
//				Kd += 0.02;
//				else if(state == Joystick::State::kDown)
//				Kd-=0.02;
//			});

    Joystick js(
    		getJoystickConfig(
    				[&Kp,&Kd](const uint8_t id, const Joystick::State state){
    		if(state == Joystick::State::kLeft)
    			Kp += 1;
    		else if(state == Joystick::State::kRight)
    			Kp -= 1;
    		else if(state == Joystick::State::kDown)
    			Kd -= 1;
    		else if(state == Joystick::State::kUp)
    			Kd+= 1;
    }
    				)
	);

	while (true) {
		while (ticks != libsc::System::Time()) {
			ticks = libsc::System::Time();
			if (ticks % 10 == 0) {
				ticks = System::Time();
				char c[15];
				const Byte* camBuffer = camera.LockBuffer();
//                extract_cam(camBuffer);
////				Print2D();
                lcdP->SetRegion(Lcd::Rect(0, 0, Width, Height));
				lcdP->FillBits(0x0000, 0xFFFF, camBuffer, Width * Height);
				camera.UnlockBuffer();
				lcdP->SetRegion(Lcd::Rect(0,Kp , Width, 2));
				lcdP->FillColor(Lcd::kRed);
				lcdP->SetRegion(Lcd::Rect(0,Kd , Width, 2));
				lcdP->FillColor(Lcd::kBlue);
				int l_count = 0;
				int r_count = 0;
				int l_correct = 0;
				int r_correct = 0;
				int l_small = 0;
				int r_small = 0;
				for(int j=Kp;j<=Kd;j++){
					for(int i=40;i<80;i++){
						if(ret_cam_bit(i, j, camBuffer) != 1){
							lcdP->SetRegion(Lcd::Rect(i,j,1,1));
							lcdP->FillColor(Lcd::kBlack);}
						else break;
					}
					for(int k=40;k>=0;k--){
						if(ret_cam_bit(k, j, camBuffer) != 1){
							lcdP->SetRegion(Lcd::Rect(k,j,1,1));
							lcdP->FillColor(Lcd::kBlack);}
						else break;
					}
				}
				for(int i=0;i<80;i++){
					for(int j=Kd;j>=Kp;j--){
						if(ret_cam_bit(i, j, camBuffer) != 1){
							lcdP->SetRegion(Lcd::Rect(i,j,1,1));
							lcdP->FillColor(Lcd::kBlack);}
						else break;
					}
				}
//				for(int j=0;j<60;j++){
//					int i=41;
//					for(;i<80 && ret_cam_bit(i, j, camBuffer) != 1;i++);
//					int k=39;
//					for(;k>=0 && ret_cam_bit(k, j, camBuffer) != 1;k--);
//					lcdP->SetRegion(Lcd::Rect(i-1,j,2,1));
//					lcdP->FillColor(Lcd::kRed);
//					lcdP->SetRegion(Lcd::Rect(k,j,2,1));
//					lcdP->FillColor(Lcd::kBlue);
//				}
				for (int j = 51; j < 55; j++) {
					int l_counter = 0;
					int r_counter = 0;
					for (int i = 39; i > 0 && ret_cam_bit(i, j, camBuffer) != 1;
							i--)
						l_counter++;
					for (int k = 41;
							k < 80 && ret_cam_bit(k, j, camBuffer) != 1; k++)
						r_counter++;
					l_count += l_counter;
					r_count += r_counter;
//                		lcdP->SetRegion(Lcd::Rect(40-l_counter-2,j-2,5,5));
//                		lcdP->FillColor(Lcd::kGreen);
//                		lcdP->SetRegion(Lcd::Rect(40+r_counter-2,j-2,5,5));
//                		lcdP->FillColor(Lcd::kCyan);
					if (l_counter > 37)
						l_correct++;
					if (r_counter > 37)
						r_correct++;
					if (l_counter < 4)
						l_small++;
					if (r_counter < 4)
						r_small++;
				}
				temp = ((r_count - l_count) * 100) / (r_count + l_count);
				if ((l_correct && r_correct) || l_small || r_small)
					temp = pre_temp;
				derivative = (-temp + pre_temp) / 10;
				Servo.SetDegree(900 - (0.87 * (-temp) + 0.8 * derivative) * 16);
//                if(((0.9*(-temp)+0.8*derivative)*16>45 || (0.9*(-temp)+0.8*derivative)*16<-45) && System::Time() - brake_ticks >= 2000){
//                		brake_ticks = System::Time();
//                		Motor.SetPower(20);
//                	    Motor.SetClockwise(true);
//                	    System::DelayMs(10);
//                	    Motor.SetPower(200);
//                	    Motor.SetClockwise(false);
//                }
//
                lcdP->SetRegion(Lcd::Rect(0,62,120,15));
                sprintf(c,"Red :%d",Kp);
                writer.WriteBuffer(c,15);
                lcdP->SetRegion(Lcd::Rect(0,79,120,15));
				sprintf(c,"Blue :%d",Kd);
				writer.WriteBuffer(c,15);
//				lcdP->SetRegion(Lcd::Rect(0,96,120,15));
//				sprintf(c,"Mag3 :%d",MagSensor3.GetResult());
//				writer.WriteBuffer(c,15);
//				lcdP->SetRegion(Lcd::Rect(0,113,120,15));
//				sprintf(c,"Mag4 :%d",MagSensor4.GetResult());
//				writer.WriteBuffer(c,15);
//				lcdP->SetRegion(Lcd::Rect(0,130,120, 15));
//				sprintf(c, "Kp: %f", Kp);
//				writer.WriteBuffer(c, 15);
//				lcdP->SetRegion(Lcd::Rect(55,130, 50, 15));
//				sprintf(c, "Kd: %f", Kd);
//				writer.WriteBuffer(c, 15);
				pre_temp = temp;
			}
		}
	}

	return 0;
}
