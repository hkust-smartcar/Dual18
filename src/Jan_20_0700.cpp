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

Led::Config getLedConfig(int led_id){
    Led::Config config;
    config.id = led_id;
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

struct Bits{
    int x, y;
    bool is_count_before;
};

int is_equal(Bits first, Bits second){
    if((first.x == second.x)&&(first.y == second.y)&&(first.is_count_before == second.is_count_before)){
        return true;
    }
    else{
        return false;
    }
}

bool see_the_right_cycle_or_not(int top_line, int bottom_line, const Byte* camBuffer){
    int right_count = 0;
    int do_not_need_to_count =0;
    vector<Bits> right_my_bit = {};
    for(int j=top_line;j<bottom_line;j++){
        for(int i=40;i<80;i++){
            if(ret_cam_bit(i, j, camBuffer) != 1){
                Bits mybit;
                mybit.x = i;
                mybit.y = j;
                mybit.is_count_before = true;
                right_my_bit.push_back(mybit);
                right_count++;
            }
            else {break;}
        }
    }

    for(int i=40;i<80;i++){
        for(int j=bottom_line-1;j>=top_line;j--){
            if(ret_cam_bit(i, j, camBuffer) != 1){
                Bits mybit2;
                mybit2.x =i;
                mybit2.y =j;
                mybit2.is_count_before = true;
                for (int i=0; i<right_my_bit.size(); i++){
                    if (is_equal(mybit2,right_my_bit[i])){
                        do_not_need_to_count++;
                        break;
                    }
                }
                right_count++;
            }
            else {break;}
        }
    }
    right_count = right_count-do_not_need_to_count;
    do_not_need_to_count = 0;


//	for(int j=top_line;j<bottom_line;j++){
//		for(int i=40;i<80;i++){
//			if(ret_cam_bit(i, j, camBuffer) == 1){
//				right_count++;
//			}
//		}
//	}
//	if(right_count<235) return true;
//	else return false;
    return right_count;
}

int see_the_left_cycle_or_not(int top_line, int bottom_line, const Byte* camBuffer){
    int left_count = 0;
    int do_not_need_to_count =0;
    vector<Bits> left_my_bit = {};
    for(int j=top_line;j<bottom_line;j++){
        for(int k=39;k>=0;k--){
            if(ret_cam_bit(k, j, camBuffer) != 1){
                Bits mybit;
                mybit.x = k;
                mybit.y = j;
                mybit.is_count_before = true;
                left_my_bit.push_back(mybit);
                left_count++;
            }
            else {
                break;
            }
        }
    }

    for(int i=0;i<40;i++){
        for(int j=bottom_line-1;j>=top_line;j--){
            if(ret_cam_bit(i, j, camBuffer) != 1){
                Bits mybit2;
                mybit2.x =i;
                mybit2.y =j;
                mybit2.is_count_before = true;
                for (int i=0; i<left_my_bit.size(); i++){
                    if (is_equal(mybit2,left_my_bit[i])){
                        do_not_need_to_count++;
                        break;
                    }
                }
                left_count++;
            }
            else {break;}
        }
    }
    left_count = left_count-do_not_need_to_count;
    do_not_need_to_count = 0;


//	for(int j=top_line;j<bottom_line;j++){
//		for(int i=0;i<40;i++){
//			if(ret_cam_bit(i, j, camBuffer) == 1){
//				left_count++;
//			}
//		}
//	}
//	if(left_count<235) return true;
//	else return false;
    return left_count;
}

int isRight(int top_line, int bottom_line,const Byte* camBuffer){
    int top_exist = 0;
    int right_exist = 0;
    for(int i=45;i<75;i++)
        if(ret_cam_bit(i,top_line,camBuffer) == 1) top_exist++;
    for(int i=top_line+1;i<bottom_line;i++)
        if(ret_cam_bit(80,i,camBuffer) == 1) right_exist++;
    if(top_exist>0 && top_exist<4 && right_exist>0 && right_exist<4)
        return true;
    else
        return false;
}
bool isLeft(int top_line, int bottom_line,const Byte* camBuffer){
    int top_exist = 0;
    int left_exist = 0;
    for(int i=5;i<35;i++)
        if(ret_cam_bit(i,top_line,camBuffer) == 1) top_exist++;
    for(int i=top_line+1;i<bottom_line;i++)
        if(ret_cam_bit(0,i,camBuffer) == 1) left_exist++;
    if(top_exist>0 && top_exist<4 && left_exist>0 && left_exist<4)
        return true;
    else
        return false;
}

int main(void) {
    System::Init();

    //	bool Motor_start = true;
    int32_t ticks = System::Time();
//	int32_t brake_ticks = System::Time();
    int pre_temp = 0;
    int temp = 0;
    float derivative;
    int top_line = 20;
    int bottom_line = 30;
    int Tstate = 0;
    int Lstate = 0;
    float Kp = 0.9;
    float Kd = 0.9;
    int left_corner = 0;
    int right_corner = 0;

    enum track_state{
        regular = 0,
        right_loop,
        left_loop,
        magnetic,
        finish
    };

    enum loop_state{
        start = 0,
        mid,
        end
    };

    k60::Ov7725 camera(getCameraConfig());
    camera.Start();

    St7735r lcd(getLcdConfig());
    lcdP = &lcd;

    LcdTypewriter writer(GetWriterConfig(&lcd));

    FutabaS3010 Servo(getServoConfig());

    Led led1(getLedConfig(0));
    Led led2(getLedConfig(1));

    AlternateMotor Motor(getMotorConfig());
    Motor.SetPower(180);
    Motor.SetClockwise(false);

    Adc MagLeft(getMagSensorConfig(Pin::Name::kPtb1));
    Adc MagRight(getMagSensorConfig(Pin::Name::kPtc10));

    Joystick js(
            getJoystickConfig(
                    [&Kp,&Kd](const uint8_t id, const Joystick::State state){
                        if(state == Joystick::State::kLeft)
                            Kp -= 0.01;
                        else if(state == Joystick::State::kRight)
                            Kp += 0.01;
                        else if(state == Joystick::State::kDown)
                            Kd += 0.01;
                        else if(state == Joystick::State::kUp)
                            Kd -= 0.01;
                    }
            )
    );

    while (true) {
        while (ticks != libsc::System::Time()) {
            ticks = libsc::System::Time();
            if (ticks % 10 == 0) {
                ticks = System::Time();

                int l_count = 0;
                int r_count = 0;
                int l_large = 0;
                int r_large = 0;
                int l_small = 0;
                int r_small = 0;
                char c[15];

                const Byte* camBuffer = camera.LockBuffer();
//                extract_cam(camBuffer);
//				Print2D();
                lcdP->SetRegion(Lcd::Rect(0, 0, Width, Height));
                lcdP->FillBits(0x0000, 0xFFFF, camBuffer, Width * Height);
                lcdP->SetRegion(Lcd::Rect(0,20 , Width, 2));
                lcdP->FillColor(Lcd::kRed);
                lcdP->SetRegion(Lcd::Rect(0,30 , Width, 2));
                lcdP->FillColor(Lcd::kBlue);



                for (int j = 51; j < 54; j++) {
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
//                		lcdP->SetRegion(Lcd::Rect(40-l_counter-1,j-1,3,3));
//                		lcdP->FillColor(Lcd::kGreen);
//                		lcdP->SetRegion(Lcd::Rect(40+r_counter-1,j-1,3,3));
//                		lcdP->FillColor(Lcd::kCyan);
                    if (l_counter > 37)
                        l_large++;
                    if (r_counter > 37)
                        r_large++;
                    if (l_counter < 4)
                        l_small++;
                    if (r_counter < 4)
                        r_small++;
                }

                temp = ((r_count - l_count) * 160) / (r_count + l_count);
                if ((l_large && r_large) || l_small || r_small)
                    temp = pre_temp;
                derivative = (-temp + pre_temp) / 10;
                if(Tstate == regular){

//					if(isRight(top_line,bottom_line,camBuffer))
                    right_corner = see_the_right_cycle_or_not(top_line, bottom_line, camBuffer);
//					if(isLeft(top_line,bottom_line,camBuffer))
                    left_corner = see_the_left_cycle_or_not(top_line, bottom_line, camBuffer);

                    if(right_corner && !left_corner){
                        Tstate = right_loop;
                        led1.Switch();
                    }
                    else if(!right_corner && left_corner)
                        Tstate = left_loop;
                    if(MagRight.GetResult()>17 || MagRight.GetResult()>17)
                        Tstate = magnetic;
                    else
                        Tstate = regular;
                }
                else if(Tstate == right_loop){
                    led2.Switch();
                    if(Lstate == loop_state::start){
                        if(!l_large){
                            temp = 40;
                            derivative = 0;
                        }
                        else if(l_large) {
                            temp = 40;
                            derivative = 0;
                            Lstate = loop_state::mid;
                        }
                    }
                    else if(Lstate == loop_state::mid){
                        if(!l_large) Lstate = loop_state::end;
                    }
                    else if(Lstate == loop_state::end){
                        if(l_large){
                            Tstate = regular;
                            Lstate = loop_state::start;
                            temp = 40;
                            derivative = 0;
                        }
                    }
                }
                else if(Tstate == left_loop){
                    if(Lstate == loop_state::start){
                        if(!r_large){
                            temp = -40;
                            derivative = 0;
                        }
                        else if(r_large) {
                            temp = -40;
                            derivative = 0;
                            Lstate = loop_state::mid;
                        }
                    }
                    else if(Lstate == loop_state::mid){
                        if(!r_large) Lstate = loop_state::end;
                    }
                    else if(Lstate == loop_state::end){
                        if(r_large){
                            Tstate = regular;
                            Lstate = loop_state::start;
                            temp = -40;
                            derivative = 0;
                        }
                    }
                }
                else if(Tstate == magnetic){
                    System::DelayS(1);
                    if(MagRight.GetResult()<16 && MagLeft.GetResult()<16){
                        Tstate = regular;
                    }
                    temp = ((MagRight.GetResult() - MagLeft.GetResult()) * 160) / (MagRight.GetResult() + MagLeft.GetResult());
                    derivative = (-temp + pre_temp) / 10;
                }
                Servo.SetDegree(900 - (1.3 * (-temp) + 0.02 * derivative) * 10);
//				if(((0.9*(-temp)+0.8*derivative)*16>40 || (0.9*(-temp)+0.8*derivative)*16<-40) && System::Time() - brake_ticks >= 3000){
//					brake_ticks = System::Time();
//					Motor.SetPower(20);
//					Motor.SetClockwise(true);
//					System::DelayMs(20);
//					Motor.SetPower(160);
//					Motor.SetClockwise(false);
//				}
                pre_temp = temp;

                lcdP->SetRegion(Lcd::Rect(0,62,120,15));
                sprintf(c,"Track :%d",Tstate);
                writer.WriteBuffer(c,15);
                lcdP->SetRegion(Lcd::Rect(0,79,120,15));
                sprintf(c,"Loop :%d",Lstate);
                writer.WriteBuffer(c,15);
                lcdP->SetRegion(Lcd::Rect(0,96,120,15));
                sprintf(c,"Right :%d",right_corner);
                writer.WriteBuffer(c,15);
                lcdP->SetRegion(Lcd::Rect(0,113,120,15));
                sprintf(c,"Left :%d",left_corner);
                writer.WriteBuffer(c,15);
//				lcdP->SetRegion(Lcd::Rect(0,130,100, 15));
//				sprintf(c, "Rcount: %d", right_corner_value);
//				writer.WriteBuffer(c, 15);
//				lcdP->SetRegion(Lcd::Rect(0,147,100, 15));
//				sprintf(c, "Lcount: %d", left_corner_value);
//				writer.WriteBuffer(c, 15);
                camera.UnlockBuffer();
            }
        }
    }

    return 0;
}
