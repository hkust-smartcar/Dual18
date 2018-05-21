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

int main() {
	System::Init();

	Led led0(GetLedConfig(0));
	Led led1(GetLedConfig(1));
	Led led2(GetLedConfig(2));
	Led led3(GetLedConfig(3));

	St7735r lcd(GetLcdConfig());
	LcdTypewriter writer(GetWriterConfig(&lcd));

	float echo0 = 0;
	float echo1 = 0;
	float echo2 = 0;
	float echo3 = 0;

//	Ultrasonic us;

	uint8_t trig = 0, pTrig = 0;

	DualCar_UART uart0(1); // << BT related

	uart0.add(DualCar_UART::FLOAT::f0, &echo0, true, 0);
	uart0.add(DualCar_UART::FLOAT::f1, &echo1, true, 0);
	uart0.add(DualCar_UART::FLOAT::f2, &echo2, true, 0);
	uart0.add(DualCar_UART::FLOAT::f3, &echo3, true, 0);

	uart0.add(DualCar_UART::UINT8_T::u0, &trig, true);

	uart0.parseValues(); // << BT related

//	SimpleBuzzer::Config config;
//	config.id = 0;
//	config.is_active_low = true;
//	SimpleBuzzer buzzzzzz(config);

//	PassiveBuzzer::Config config;
//	PassiveBuzzer buzzzerrrrrrrr(config);
//	buzzzerrrrrrrr.SetBeep(true);

	uint32_t lastTime = 0;
	while (true) {
		if (System::Time() != lastTime) {
			lastTime = System::Time();
			if (lastTime % 100 == 0) {
				led0.Switch();
				led1.Switch();
				led2.Switch();
				led3.Switch();

				echo0 += 1;

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
			}

			uart0.RunEveryMS(); // << BT related
		}
	}
}
