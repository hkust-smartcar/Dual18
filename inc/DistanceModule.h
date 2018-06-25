/*
 * DistanceModule.h
 *
 *  Created on: 17 Jun 2018
 *      Author: JosephYim
 *
 *
 *      DistanceModule d([&] (float distanceInCm) {
 *
 *      	// avoid func to be used here
		if (distanceInCm > 30) {
			led2.SetEnable(true);
		} else {
			led2.SetEnable(false);
		}
	});

 */

#ifndef INC_DISTANCEMODULE_H_
#define INC_DISTANCEMODULE_H_

#include "libbase/k60/cmsis/mk60f15.h"
#include <libbase/k60/mcg.h>
#include <libsc/system.h>
#include "libbase/k60/pit.h"
#include "libbase/k60/vectors.h"

#include <cstddef>
#include <cstdint>
#include <functional>
#include <libsc/system.h>

using libsc::System;
using namespace libsc;
using namespace libbase::k60;

class DistanceModule {
public:
	typedef std::function<void(float distanceInCm)> TRIGG;

	DistanceModule(
			TRIGG f_
			) :
			m_trig(getGpoConfig(0)), m_echo(getGpiConfig(0)), m_echo_listener([&](Gpi *gpi) {
				m_distance = (sentPeriod * 150000 - m_pit.GetCountLeft()) * 34 / 2 / 150000;
				m_trig_func(m_distance);

			}), m_pit_listener([&](Pit *pit) {
				if (m_trig.Get() == true) {
					m_distance = 888888;
					m_trig.Set(false);
					m_pit.SetCount(150000 * sentPeriod);
				} else {
					m_trig.Set(true);
					m_pit.SetCount(150000 * 5);
				}

				debug_runTime++;

			}), m_pit(getPitConfig(m_pit_listener)), m_distance(0) {

		m_trig_func = f_;
		debug_runTime = 0;
		NVIC_SetPriority(PORTC_IRQn, __BASE_IRQ_PRIORITY - 2);
	}

	float getDistance(uint8_t id = 0) {
		// 340 m/s ~ 340 / 60 m/ms ~ 340 /60 / 8 m/125us ~ 0.7
		return m_distance;
	}

	int debug_runTime;

private:
	const int sentPeriod = 100;
	const Pin::Name TRIG_PIN = Pin::Name::kPtc14;
	const Pin::Name ECHO_PIN = Pin::Name::kPtc15;

	typedef std::function<void(Gpi *gpi)> Listener;
	Listener m_echo_listener;

	TRIGG m_trig_func;

	Gpo m_trig;
	Gpi m_echo;

	Pit::OnPitTriggerListener m_pit_listener;
	Pit m_pit;

	float m_distance;

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
		config.is_high = true;
		config.pin = TRIG_PIN;
		config.config.set(Pin::Config::ConfigBit::kHighDriveStrength);
		return config;

	}
};

#endif /* INC_DISTANCEMODULE_H_ */
