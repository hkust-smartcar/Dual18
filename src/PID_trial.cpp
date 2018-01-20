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
    Motor.SetPower(175);
    Motor.SetClockwise(false);

    int32_t ticks = System::Time();
    int pre_temp = 0;
    int temp = 0;
    float derivative;

    while (true) {
        while (ticks != libsc::System::Time()) {
            ticks = libsc::System::Time();
            if (ticks % 8 == 0) {
                ticks = System::Time();
                Motor.SetPower(175);
//				char c[15];
                const Byte* camBuffer = camera.LockBuffer();
//                lcdP->SetRegion(Lcd::Rect(0, 0, Width, Height));
//				lcdP->FillBits(0x0000, 0xFFFF, camBuffer, Width * Height);
                camera.UnlockBuffer();
                int l_count = 0;
                int r_count = 0;
                int l_correct = 0;
                int r_correct = 0;
                int l_small = 0;
                int r_small = 0;
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
                    if (l_counter > 37)
                        l_correct++;
                    if (r_counter > 37)
                        r_correct++;
                    if (l_counter < 4)
                        l_small++;
                    if (r_counter < 4)
                        r_small++;
                }
                temp = ((r_count - l_count) * 90) / (r_count + l_count);
                if ((l_correct && r_correct) || l_small || r_small)
                    temp = pre_temp;
                derivative = (-temp + pre_temp) / 10;
                if(temp<25 && temp>-25){
                    Servo.SetDegree(900 - (0.825 * (-temp) + 0.25 * derivative) * 10);
                }
                else if(temp<40 && temp>-40){
                    Servo.SetDegree(900 - (1.2 * (-temp) + 0 * derivative) * 10);
                    Motor.SetPower(130);
                }
                else{
                    Servo.SetDegree(900 - (1.405 * (-temp) + 0 * derivative) * 10);
                    Motor.SetPower(130);
                }
                pre_temp = temp;
            }
        }
    }

    return 0;
}
