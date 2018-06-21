#include <cmath>
#include <vector>
#include <cassert>
#include <cstring>
#include <string>

#include <libbase/k60/mcg.h>
#include <libsc/system.h>
#include <libbase/k60/uart.h>
#include "libsc/led.h"
#include "libsc/button.h"
#include "libsc/lcd_typewriter.h"
#include "libsc/st7735r.h"
#include "libsc/k60/ov7725.h"
#include "libsc/joystick.h"
#include "libbase/k60/adc.h"

#include "libsc/alternate_motor.h"
#include "libsc/encoder.h"
#include "libsc/futaba_s3010.h"
#include "libsc/dir_encoder.h"

#include "DualCar_UART.h"
#include "DistanceModule.h"
#include "Buzzer.h"

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
using libsc::k60::Ov7725;

Led::Config GetLedConfig(uint8_t id) {
	Led::Config config;
	config.id = id;
	config.is_active_low = true;
	return config;
}

LcdTypewriter::Config GetWriterConfig(St7735r *lcd) {
	LcdTypewriter::Config config;
	config.lcd = lcd;
	return config;
}

St7735r::Config GetLcdConfig() {
	St7735r::Config config;
	config.fps = 20;
	return config;
}

Adc::Config getADCConfig(uint8_t id) {
	Adc::Config config;
	config.speed = Adc::Config::SpeedMode::kFast;
	config.is_continuous_mode = false;
	config.is_diff_mode = true;
	config.avg_pass = Adc::Config::AveragePass::kDisable;
	config.resolution = Adc::Config::Resolution::k8Bit;
	switch (id) {
		case 0:
			config.adc = Adc::Name::kAdc0Ad4B;
//			config.pin = libbase::k60::Pin::Name::kPtc2;
			break;
		case 1:
			config.adc = Adc::Name::kAdc1Ad4B;
//			config.pin = libbase::k60::Pin::Name::kPtc8;
			break;
		case 2:
			config.adc = Adc::Name::kAdc1Ad5B;
//			config.pin = libbase::k60::Pin::Name::kPtc9;
			break;
		case 3:
			config.adc = Adc::Name::kAdc1Ad6B;
//			config.pin = libbase::k60::Pin::Name::kPtc10;
			break;
		case 4:
			config.adc = Adc::Name::kAdc1Ad7B;
//			config.pin = libbase::k60::Pin::Name::kPtc11;
			break;
		case 5:
			config.adc = Adc::Name::kAdc0Ad5B;
//			config.pin = libbase::k60::Pin::Name::kPtd1;
			break;
		case 6:
			config.adc = Adc::Name::kAdc0Ad6B;
//			config.pin = libbase::k60::Pin::Name::kPtd5;
			break;
//		case 7:
//			config.adc = Adc::Name::kAdc0Ad7B;
//			config.pin = libbase::k60::Pin::Name::kPtd6;
//			break;
		default:
			break;
	}
	return config;
}

DirEncoder::Config GetEncoderConfig(int m_id) {
	DirEncoder::Config config;
	config.id = m_id;
	return config;
}

FutabaS3010::Config GetServoConfig() {
	FutabaS3010::Config config;
	config.id = 0;
	return config;
}

AlternateMotor::Config GetMotorConfig(int id) {
	AlternateMotor::Config config;
	config.id = id;
	config.multiplier = 100;
	return config;
}

int main() {
	System::Init();

	Led led0(GetLedConfig(0));
	Led led1(GetLedConfig(1));
//	Led led2(GetLedConfig(2));
//	Led led3(GetLedConfig(3));

	St7735r lcd(GetLcdConfig());
	LcdTypewriter writer(GetWriterConfig(&lcd));
	lcd.SetRegion(Lcd::Rect(0, 0, 80, 60));
//
//	Adc adc0(getADCConfig(0));
//	Adc adc1(getADCConfig(1));
//	Adc adc2(getADCConfig(2));
//	Adc adc3(getADCConfig(3));
//	Adc adc4(getADCConfig(4));
//	Adc adc5(getADCConfig(5));
//	Adc adc6(getADCConfig(6));
//	Adc adc7(getADCConfig(7));

//	adc0.StartConvert();
//	adc1.StartConvert();
//	adc2.StartConvert();
//	adc3.StartConvert();
//	adc4.StartConvert();
//	adc5.StartConvert();
//	adc6.StartConvert();
//	adc7.StartConvert();

//	FutabaS3010 servo(GetServoConfig());
//	AlternateMotor right_motor(GetMotorConfig(0));
//	AlternateMotor left_motor(GetMotorConfig(1));
//	DirEncoder dirEncoder0(GetEncoderConfig(0));
//	DirEncoder dirEncoder1(GetEncoderConfig(1));

//	int t = 250;
//	servo.SetDegree(900);
//	right_motor.SetPower(t);
//	left_motor.SetPower(t);

	DualCar_UART uart0(1); // << BT related
	uart0.parseValues(); // << BT related

//	musicPlayer player;

	char str[10] = "";
//	Joystick::Listener jDispatcher = [&](const uint8_t id, const Joystick::State which) {
//		if (id == 0)
//		switch (which) {
//			case Joystick::State::kDown:
//			sprintf(str, "down");
//			break;
//			case Joystick::State::kLeft:
//			sprintf(str, "left");
//			break;
//			case Joystick::State::kRight:
//			sprintf(str, "right");
//			break;
//			case Joystick::State::kSelect:
//			sprintf(str, "select");
//			break;
//			case Joystick::State::kUp:
//			sprintf(str, "up");
//			break;
//			default:
//			break;
//		}
//	};

//	Joystick::Config jCon;
//	jCon.id = 0;
//	jCon.is_active_low = true;
//	jCon.dispatcher = jDispatcher;
//	jCon.listener_triggers[0] = Joystick::Config::Trigger::kDown;
//	jCon.listener_triggers[1] = Joystick::Config::Trigger::kDown;
//	jCon.listener_triggers[2] = Joystick::Config::Trigger::kDown;
//	jCon.listener_triggers[3] = Joystick::Config::Trigger::kDown;
//	jCon.listener_triggers[4] = Joystick::Config::Trigger::kDown;
//	Joystick jStick(jCon);

//	Ov7725::Config con;
//	con.h = 60;
//	con.w = 80;
//	con.id = 0;
//	con.fps = Ov7725::Config::Fps::kHigh;

	DistanceModule d;

//	Ov7725 cam(con);
//	cam.Start();

	uint32_t lastTime = 0;
	while (true) {
		if (System::Time() != lastTime) {
			lastTime = System::Time();
			if (lastTime % 100 == 0) {
//				lcd.SetRegion(Lcd::Rect(0, 0, 80, 60));
//				const Byte* Buffer = cam.LockBuffer();
//				lcd.FillBits(Lcd::kBlack, Lcd::kWhite, Buffer, 60 * 80);
//				cam.UnlockBuffer();
//
//				lcd.SetRegion(Lcd::Rect(0, 60, 80, 15));
//				lcd.FillColor(Lcd::kBlack);
//				writer.WriteBuffer(str, 10);

//				uart0.Send_uint8_t(DualCar_UART::UINT8_T::u0, adc0.GetResult());
//				uart0.Send_uint8_t(DualCar_UART::UINT8_T::u1, adc1.GetResult());
//				uart0.Send_uint8_t(DualCar_UART::UINT8_T::u2, adc2.GetResult());
//				uart0.Send_uint8_t(DualCar_UART::UINT8_T::u3, adc3.GetResult());
//				uart0.Send_uint8_t(DualCar_UART::UINT8_T::u4, adc4.GetResult());
//				uart0.Send_uint8_t(DualCar_UART::UINT8_T::u5, adc5.GetResult());
//				uart0.Send_uint8_t(DualCar_UART::UINT8_T::u6, adc6.GetResult());
//				uart0.Send_uint8_t(DualCar_UART::UINT8_T::u7, adc7.GetResult());

				uart0.Send_int(DualCar_UART::INT::i0, d.getDistance());
				uart0.Send_int(DualCar_UART::INT::i1, d.debug_runTime);

//				lcd.SetRegion(Lcd::Rect(0, 60, 80, 15));
//				char c[20];
//				sprintf(c, "u5s: %d    ", adc5.GetResult());
//				writer.WriteBuffer(c, 10);

//				uart0.Send_bool(DualCar_UART::BOOLEAN::b0, playBuzzer);

				led0.Switch();
				led1.Switch();
//				led2.Switch();
//				led3.Switch();

//				echo0 += 1;

//				echo0 = (float) us.getTime(0);
//				echo1 = (float) us.getTime(1);
//				echo2 = (float) us.getTime(2);
//				echo3 = (float) us.getTime(3);

			}

			uart0.RunEveryMS(); // << BT related
		}
	}
}
