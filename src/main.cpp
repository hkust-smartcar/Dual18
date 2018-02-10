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
#include "config.h"
using libsc::System;
using namespace libsc;
using namespace libbase::k60;
using namespace std;

Mcg::Config Mcg::GetMcgConfig() {
            Mcg::Config config;
            config.external_oscillator_khz = 50000;
            config.core_clock_khz = 150000;
            return config;
}


bool camptr[Height][Width];

inline bool ret_cam_bit(int x, int y, const Byte* camBuffer) {
    return ((camBuffer[y * 10 + x / 8] >> (7 - (x % 8))) & 1);
}

St7735r* lcdP;

int main(void) {
    System::Init();

    int32_t ticks = System::Time();

	Led led1(getLedConfig(0));
	Led led2(getLedConfig(1));
	Led led3(getLedConfig(2));
	char c[15];


    k60::Ov7725 camera(getCameraConfig());
    camera.Start();

    St7735r lcd(getLcdConfig());
    lcdP = &lcd;

    LcdTypewriter writer(GetWriterConfig(&lcd));

    FutabaS3010 Servo(getServoConfig());

    AlternateMotor Motor(getMotorConfig());
    Motor.SetPower(155);
    Motor.SetClockwise(false);

    Adc MagLeft(getMagSensorConfig(Pin::Name::kPtc11));
    Adc MagRight(getMagSensorConfig(Pin::Name::kPtc10));

    while (true) {
        while (ticks != libsc::System::Time()) {
            ticks = libsc::System::Time();
            if (ticks % 10 == 0) {
                ticks = System::Time();

                const Byte* camBuffer = camera.LockBuffer();
                lcdP->SetRegion(Lcd::Rect(0, 0, Width, Height));
                lcdP->FillBits(0x0000, 0xFFFF, camBuffer, Width * Height);


//				lcdP->SetRegion(Lcd::Rect(0,130,100, 15));
//				sprintf(c, "temp: %d", temp);
//				writer.WriteBuffer(c, 15);

            }
        }
    }

    return 0;
}
