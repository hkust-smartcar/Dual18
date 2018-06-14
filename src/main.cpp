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

#include "libsc/simple_buzzer.h"
#include "libsc/passive_buzzer.h"

#include "DualCar_UART.h"

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
	config.speed = Adc::Config::SpeedMode::kExFast;
	config.is_continuous_mode = true;
	config.avg_pass = Adc::Config::AveragePass::k16;
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
		case 7:
			config.adc = Adc::Name::kAdc0Ad7B;
//			config.pin = libbase::k60::Pin::Name::kPtd6;
			break;
		default:
			break;
	}
	return config;
}

#include <cstddef>
#include <cstdint>
#include <functional>
#include <libsc/system.h>

#include LIBBASE_H(gpio)

class Ultrasonic {
public:
	Ultrasonic() {
		// init trig
		m_trig_pins[0] = libbase::k60::Pin::Name::kPtb16;
		m_trig_pins[1] = libbase::k60::Pin::Name::kPtb18;
		m_trig_pins[2] = libbase::k60::Pin::Name::kPtb20;
		m_trig_pins[3] = libbase::k60::Pin::Name::kPtb22;

		Gpo::Config GpoConfig;
		GpoConfig.is_high = false;

		for (uint8_t t = 0; t < m_TRIG_PIN_NUM; t++) {
			GpoConfig.pin = m_trig_pins[t];
			m_trig[t] = Gpo(GpoConfig);
		};

		// init listener
		for (uint8_t t = 0; t < m_ECHO_PIN_NUM; t++) {
			m_echo_listener[t] = [&](Gpi *gpi) {
				m_echo_time_125us[t] = System::TimeIn125us() - m_echo_time_125us[t];
			};
		}

		m_echo_pins[0] = libbase::k60::Pin::Name::kPtb17;
		m_echo_pins[1] = libbase::k60::Pin::Name::kPtb19;
		m_echo_pins[2] = libbase::k60::Pin::Name::kPtb21;
		m_echo_pins[3] = libbase::k60::Pin::Name::kPtb23;

		Gpi::Config GpiConfig;
		GpiConfig.config.set(Pin::Config::ConfigBit::kPullEnable);
		GpiConfig.interrupt = Pin::Config::Interrupt::kFalling;

		for (uint8_t t = 0; t < m_ECHO_PIN_NUM; t++) {
			GpiConfig.isr = m_echo_listener[t];
			GpiConfig.pin = m_echo_pins[t];
			m_echo[t] = Gpi(GpiConfig);
			m_echo_time_125us[t] = 0;
		}
	}

	void sendPulse() {
		resetTimer();

		for (uint8_t t = 0; t < m_TRIG_PIN_NUM; t++) {
			m_trig[t].Set(true);
		}
		System::DelayMs(1);
		for (uint8_t t = 0; t < m_TRIG_PIN_NUM; t++) {
			m_trig[t].Set(false);
		}
	}

	uint32_t getTime(uint8_t id) {
		return (uint32_t) (m_echo_time_125us[id]);
		// 340 m/s ~ 340 / 60 m/ms ~ 340 /60 / 8 m/125us ~ 0.7
	}

private:
	typedef std::function<void(Gpi *gpi)> Listener;

	LIBBASE_MODULE(Gpo) m_trig[4];
	Pin::Name m_trig_pins[4];

	LIBBASE_MODULE(Gpi) m_echo[4];
	Pin::Name m_echo_pins[4];
	Listener m_echo_listener[4];

	uint32_t m_echo_time_125us[4];
	const uint8_t m_ECHO_PIN_NUM = (sizeof(m_echo_pins) / sizeof(*m_echo_pins));
	const uint8_t m_TRIG_PIN_NUM = (sizeof(m_trig_pins) / sizeof(*m_trig_pins));

	inline void resetTimer() {
		for (uint8_t t = 0; t < m_ECHO_PIN_NUM; t++) {
			m_echo_time_125us[t] = System::TimeIn125us();
		}
	}
};

#include <libutil/notes.h>
#include <math.h>
#include <vector>
using std::vector;
class musicPlayer {
public:
	enum note {
		none, c, cs, d, ds, e, f, fs, g, gs, a, as, b
	};

	struct node {
		note n;
		uint8_t octal;
		float duration;
		node(note _n, uint8_t _octal, float _d) :
				n(_n), octal(_octal), duration(_d) {
		}
	};

	musicPlayer() :
			buzzzer(getBuzzerConfig()) {
		playSong(NokiaSound);
	}

	PassiveBuzzer buzzzer;
	float multiplier = 60 * 1000 / 150;

	uint8_t t = 4;
	std::vector<musicPlayer::node> NokiaSound = { { e, t, 0.5 }, { d, t, 0.5 }, { fs, t, 1 }, { gs, t, 1 }, { c, t,
			0.5 }, { b, t, 0.5 }, { d, t, 1 }, { e, t, 1 }, { b, t, 0.5 }, { a, t, 0.5 }, { cs, t, 1 },
			{ e, t, 1 }, { a, t, 3 } };

	void playSong(std::vector<node> song) {
		for (auto &t : song) {
			buzzzer.SetNote(getFreq(t.n, t.octal));
			System::DelayMs(t.duration * multiplier);
			buzzzer.SetBeep(true);
		}
		buzzzer.SetBeep(false);
	}

	const float c_freq = 4186.01;
	const float cs_freq = 4434.92;
	const float d_freq = 4698.63;
	const float ds_freq = 4978.03;
	const float e_freq = 5274.04;
	const float f_freq = 5587.65;
	const float fs_freq = 5919.91;
	const float g_freq = 6271.93;
	const float gs_freq = 6644.88;
	const float a_freq = 7040.00;
	const float as_freq = 7458.62;
	const float b_freq = 7901.13;

	float getFreq(note &n, uint8_t &octal) {
		switch (n) {
			case note::c:
				return (c_freq / pow(2, 8 - octal));
				break;
			case note::cs:
				return (cs_freq / pow(2, 8 - octal));
				break;
			case note::d:
				return (d_freq / pow(2, 8 - octal));
				break;
			case note::ds:
				return (ds_freq / pow(2, 8 - octal));
				break;
			case note::e:
				return (e_freq / pow(2, 8 - octal));
				break;
			case note::f:
				return (f_freq / pow(2, 8 - octal));
				break;
			case note::fs:
				return (fs_freq / pow(2, 8 - octal));
				break;
			case note::g:
				return (g_freq / pow(2, 8 - octal));
				break;
			case note::gs:
				return (gs_freq / pow(2, 8 - octal));
				break;
			case note::a:
				return (a_freq / pow(2, 8 - octal));
				break;
			case note::as:
				return (as_freq / pow(2, 8 - octal));
				break;
			case note::b:
				return (b_freq / pow(2, 8 - octal));
				break;
			default:
				return 0;
		};

	}

	PassiveBuzzer::Config getBuzzerConfig() {
		PassiveBuzzer::Config config;
		config.id = 0;
		return config;
	}
};

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

#include "libbase/k60/cmsis/mk60f15.h"
#include "libbase/k60/flash.h"

int main() {
	System::Init();

	Led led0(GetLedConfig(0));
	Led led1(GetLedConfig(1));
	Led led2(GetLedConfig(2));
	Led led3(GetLedConfig(3));

	St7735r lcd(GetLcdConfig());
	LcdTypewriter writer(GetWriterConfig(&lcd));
	lcd.SetRegion(Lcd::Rect(0, 0, 80, 60));

	Adc adc0(getADCConfig(0));
	Adc adc1(getADCConfig(1));
	Adc adc2(getADCConfig(2));
	Adc adc3(getADCConfig(3));
	Adc adc4(getADCConfig(4));
	Adc adc5(getADCConfig(5));
	Adc adc6(getADCConfig(6));
	Adc adc7(getADCConfig(7));

	adc0.StartConvert();
	adc1.StartConvert();
	adc2.StartConvert();
	adc3.StartConvert();
	adc4.StartConvert();
	adc5.StartConvert();
	adc6.StartConvert();
	adc7.StartConvert();

//	char t[10] = "123456789";
//	writer.WriteBuffer(t, 10);

//	float echo0 = 0;
//	float echo1 = 0;
//	float echo2 = 0;
//	float echo3 = 0;

	FutabaS3010 servo(GetServoConfig());
//	AlternateMotor right_motor(GetMotorConfig(0));
//	AlternateMotor left_motor(GetMotorConfig(1));
//	DirEncoder dirEncoder0(GetEncoderConfig(0));
//	DirEncoder dirEncoder1(GetEncoderConfig(1));

//	int t = 250;
	servo.SetDegree(900);
//	right_motor.SetPower(t);
//	left_motor.SetPower(t);

//	Ultrasonic us;

	uint8_t trig = 0, pTrig = 0;
	int i0 = 0, i1 = 0;

	DualCar_UART uart0(1); // << BT related

//	uart0.add(DualCar_UART::FLOAT::f0, &echo0, true, 0);
//	uart0.add(DualCar_UART::FLOAT::f1, &echo1, true, 0);
//	uart0.add(DualCar_UART::FLOAT::f2, &echo2, true, 0);
//	uart0.add(DualCar_UART::FLOAT::f3, &echo3, true, 0);

	uart0.add(DualCar_UART::UINT8_T::u0, &trig, true);
//	uart0.add(DualCar_UART::INT::i0, &i0, true);
//	uart0.add(DualCar_UART::INT::i1, &i1, true);

	uart0.parseValues(); // << BT related

//	SimpleBuzzer::Config config;
//	config.id = 0;
//	config.is_active_low = true;
//	SimpleBuzzer buzzzzzz(config);

//	PassiveBuzzer::Config config;
//	PassiveBuzzer buzzzerrrrrrrr(config);
//	buzzzerrrrrrrr.SetBeep(true);

//	musicPlayer player;

//	while (true);

	char str[10] = "null";
	Joystick::Listener jDispatcher = [&](const uint8_t id, const Joystick::State which) {
		if (id == 0)
		switch (which) {
			case Joystick::State::kDown:
			sprintf(str, "down");
			break;
			case Joystick::State::kLeft:
			sprintf(str, "left");
			break;
			case Joystick::State::kRight:
			sprintf(str, "right");
			break;
			case Joystick::State::kSelect:
			sprintf(str, "select");
			break;
			case Joystick::State::kUp:
			sprintf(str, "up");
			break;
			default:
			break;
		}
	};

	Joystick::Config jCon;
	jCon.id = 0;
	jCon.is_active_low = true;
	jCon.dispatcher = jDispatcher;
	jCon.listener_triggers[0] = Joystick::Config::Trigger::kDown;
	jCon.listener_triggers[1] = Joystick::Config::Trigger::kDown;
	jCon.listener_triggers[2] = Joystick::Config::Trigger::kDown;
	jCon.listener_triggers[3] = Joystick::Config::Trigger::kDown;
	jCon.listener_triggers[4] = Joystick::Config::Trigger::kDown;
	Joystick jStick(jCon);

	Ov7725::Config con;
	con.h = 60;
	con.w = 80;
	con.id = 0;
	con.fps = Ov7725::Config::Fps::kHigh;

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

//				lcd.SetRegion(Lcd::Rect(0, 60, 80, 15));
//				lcd.FillColor(Lcd::kBlack);
//				writer.WriteBuffer(str, 10);

				char str1[10] = "null";
//				dirEncoder0.Update();
//				int e = dirEncoder0.GetCount();
//				sprintf(str1, "%d", e);
//				lcd.SetRegion(Lcd::Rect(0, 75, 80, 15));
//				lcd.FillColor(Lcd::kBlack);
//				writer.WriteBuffer(str1, 10);

//				sprintf(str1, "i0: %d", i0);
//				lcd.SetRegion(Lcd::Rect(0, 90, 80, 15));
//				lcd.FillColor(Lcd::kBlack);
//				writer.WriteBuffer(str1, 10);
//
//				sprintf(str1, "i1: %d", i1);
//				lcd.SetRegion(Lcd::Rect(0, 105, 80, 15));
//				lcd.FillColor(Lcd::kBlack);
//				writer.WriteBuffer(str1, 10);

				int t = adc0.GetResult();
				sprintf(str1, "0: %d", t);
				lcd.SetRegion(Lcd::Rect(0, 0, 80, 15));
				lcd.FillColor(Lcd::kBlack);
				writer.WriteBuffer(str1, 10);

				t = adc1.GetResult();
				sprintf(str1, "1: %d", t);
				lcd.SetRegion(Lcd::Rect(0, 15, 80, 15));
				lcd.FillColor(Lcd::kBlack);
				writer.WriteBuffer(str1, 10);

				t = adc2.GetResult();
				sprintf(str1, "2: %d", t);
				lcd.SetRegion(Lcd::Rect(0, 30, 80, 15));
				lcd.FillColor(Lcd::kBlack);
				writer.WriteBuffer(str1, 10);

				t = adc3.GetResult();
				sprintf(str1, "3: %d", t);
				lcd.SetRegion(Lcd::Rect(0, 45, 80, 15));
				lcd.FillColor(Lcd::kBlack);
				writer.WriteBuffer(str1, 10);

				t = adc4.GetResult();
				sprintf(str1, "4: %d", t);
				lcd.SetRegion(Lcd::Rect(0, 60, 80, 15));
				lcd.FillColor(Lcd::kBlack);
				writer.WriteBuffer(str1, 10);

				t = adc5.GetResult();
				sprintf(str1, "5: %d", t);
				lcd.SetRegion(Lcd::Rect(0, 75, 80, 15));
				lcd.FillColor(Lcd::kBlack);
				writer.WriteBuffer(str1, 10);

				t = adc6.GetResult();
				sprintf(str1, "6: %d", t);
				lcd.SetRegion(Lcd::Rect(0, 90, 80, 15));
				lcd.FillColor(Lcd::kBlack);
				writer.WriteBuffer(str1, 10);

				t = adc7.GetResult();
				sprintf(str1, "7: %d", t);
				lcd.SetRegion(Lcd::Rect(0, 105, 80, 15));
				lcd.FillColor(Lcd::kBlack);
				writer.WriteBuffer(str1, 10);

				led0.Switch();
				led1.Switch();
				led2.Switch();
				led3.Switch();

//				echo0 += 1;

//				echo0 = (float) us.getTime(0);
//				echo1 = (float) us.getTime(1);
//				echo2 = (float) us.getTime(2);
//				echo3 = (float) us.getTime(3);

			}

			if (trig != pTrig) {
//				us.sendPulse();
				pTrig = trig;
			}

			if (lastTime % 1000 == 0) {
				trig++;
				i0++;
			}

			uart0.RunEveryMS(); // << BT related
		}
	}
}
