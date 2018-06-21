/*
 * DistanceModule.h
 *
 *  Created on: 17 Jun 2018
 *      Author: JosephYim
 */

#ifndef INC_DISTANCEMODULE_H_
#define INC_DISTANCEMODULE_H_

#include "libbase/k60/cmsis/mk60f15.h"
#include <libbase/k60/mcg.h>
#include <libsc/system.h>
//#include <libbase/k60/uart.h>
//#include "libsc/led.h"
//#include "libsc/button.h"
//#include "libsc/lcd_typewriter.h"
//#include "libsc/st7735r.h"
//#include "libsc/k60/ov7725.h"
//#include "libsc/joystick.h"
//#include "libbase/k60/adc.h"
#include "libbase/k60/pit.h"
#include "libbase/k60/vectors.h"

#include <cstddef>
#include <cstdint>
#include <functional>
#include <libsc/system.h>

//class DistanceModule {
//public:
//	DistanceModule() {
//		// init trig
//		m_trig_pins[0] = libbase::k60::Pin::Name::kPtb16;
//		m_trig_pins[1] = libbase::k60::Pin::Name::kPtb18;
//		m_trig_pins[2] = libbase::k60::Pin::Name::kPtb20;
//		m_trig_pins[3] = libbase::k60::Pin::Name::kPtb22;
//
//		Gpo::Config GpoConfig;
//		GpoConfig.is_high = false;
//
//		for (uint8_t t = 0; t < m_TRIG_PIN_NUM; t++) {
//			GpoConfig.pin = m_trig_pins[t];
//			m_trig[t] = Gpo(GpoConfig);
//		};
//
//		// init listener
//		for (uint8_t t = 0; t < m_ECHO_PIN_NUM; t++) {
//			m_echo_listener[t] = [&](Gpi *gpi) {
//				m_echo_time_125us[t] = System::TimeIn125us() - m_echo_time_125us[t];
//			};
//		}
//
//		m_echo_pins[0] = libbase::k60::Pin::Name::kPtb17;
//		m_echo_pins[1] = libbase::k60::Pin::Name::kPtb19;
//		m_echo_pins[2] = libbase::k60::Pin::Name::kPtb21;
//		m_echo_pins[3] = libbase::k60::Pin::Name::kPtb23;
//
//		Gpi::Config GpiConfig;
//		GpiConfig.config.set(Pin::Config::ConfigBit::kPullEnable);
//		GpiConfig.interrupt = Pin::Config::Interrupt::kFalling;
//
//		for (uint8_t t = 0; t < m_ECHO_PIN_NUM; t++) {
//			GpiConfig.isr = m_echo_listener[t];
//			GpiConfig.pin = m_echo_pins[t];
//			m_echo[t] = Gpi(GpiConfig);
//			m_echo_time_125us[t] = 0;
//		}
//	}
//
//	void sendPulse() {
//		resetTimer();
//
//		for (uint8_t t = 0; t < m_TRIG_PIN_NUM; t++) {
//			m_trig[t].Set(true);
//		}
//		System::DelayMs(1);
//		for (uint8_t t = 0; t < m_TRIG_PIN_NUM; t++) {
//			m_trig[t].Set(false);
//		}
//	}
//
//	uint32_t getTime(uint8_t id) {
//		return (uint32_t) (m_echo_time_125us[id]);
//		// 340 m/s ~ 340 / 60 m/ms ~ 340 /60 / 8 m/125us ~ 0.7
//	}
//
//private:
//	typedef std::function<void(Gpi *gpi)> Listener;
//
//	LIBBASE_MODULE(Gpo) m_trig[4];
//	Pin::Name m_trig_pins[4];
//
//	LIBBASE_MODULE(Gpi) m_echo[4];
//	Pin::Name m_echo_pins[4];
//	Listener m_echo_listener[4];
//
//	uint32_t m_echo_time_125us[4];
//	const uint8_t m_ECHO_PIN_NUM = (sizeof(m_echo_pins) / sizeof(*m_echo_pins));
//	const uint8_t m_TRIG_PIN_NUM = (sizeof(m_trig_pins) / sizeof(*m_trig_pins));
//
//	inline void resetTimer() {
//		for (uint8_t t = 0; t < m_ECHO_PIN_NUM; t++) {
//			m_echo_time_125us[t] = System::TimeIn125us();
//		}
//	}
//};

using libsc::System;
using namespace libsc;
using namespace libbase::k60;

class DistanceModule {
public:
	DistanceModule() :
			m_trig(getGpoConfig(0)), m_echo(getGpiConfig(0)), m_echo_listener([&](Gpi *gpi) {
//				m_distance = m_pit.GetCountLeft() / 150000;
					m_distance = System::TimeIn125us() - sentTime;

				}), m_pit_listener([&](Pit *pit) {
				if (m_trig.Get() == true) {
					m_trig.Set(false);
					m_pit.SetCount(150000 * 1000);

					sentTime = System::TimeIn125us();
				} else {
					m_trig.Set(true);
					m_pit.SetCount(150000 * 5);
				}

				debug_runTime++;

			}), m_pit(getPitConfig(m_pit_listener)), m_distance(0) {

		debug_runTime = 0;
		NVIC_SetPriority(PORTC_IRQn, __BASE_IRQ_PRIORITY - 1);
	}

	int getDistance(uint8_t id = 0) {
		// 340 m/s ~ 340 / 60 m/ms ~ 340 /60 / 8 m/125us ~ 0.7
		return m_distance;
	}

	int debug_runTime;

private:
	const Pin::Name TRIG_PIN = Pin::Name::kPtc14;
	const Pin::Name ECHO_PIN = Pin::Name::kPtc15;

	typedef std::function<void(Gpi *gpi)> Listener;
	Listener m_echo_listener;

	Gpo m_trig;
	Gpi m_echo;

	Pit::OnPitTriggerListener m_pit_listener;
	Pit m_pit;

	int m_distance;
	int sentTime;

	Pit::Config getPitConfig(Pit::OnPitTriggerListener isr) {
		Pit::Config config;
		config.channel = 2;
		config.is_enable = true;
		config.isr = isr;
		config.count = 10 * 150000;
		return config;
	}

	Gpi::Config getGpiConfig(uint8_t id) {

		Gpi::Config config;
		config.config.set(Pin::Config::ConfigBit::kPullEnable);
		config.interrupt = Pin::Config::Interrupt::kFalling;
		config.isr = m_echo_listener;
		config.pin = ECHO_PIN;
		return config;

	}

	Gpo::Config getGpoConfig(uint8_t id) {

		Gpo::Config config;
		config.is_high = false;
		config.pin = TRIG_PIN;
		config.config.set(Pin::Config::ConfigBit::kHighDriveStrength);
		return config;

	}
};

#endif /* INC_DISTANCEMODULE_H_ */
