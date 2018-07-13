/*
 * BoardID.h
 *
 *  Created on: 25 Jun 2018
 *      Author: JosephYim
 *
 *      e.g.
 *      BoardID on9;
 *
     				if (on9.isBoardA()) {
					uart0.Send_int(DualCar_UART::INT::i5, 10);
					led2.SetEnable(true);
				} else {
					uart0.Send_int(DualCar_UART::INT::i5, 0);
					led2.SetEnable(false);
				}

				if (on9.isCar1()) {
					uart0.Send_int(DualCar_UART::INT::i6, 10);
					led3.SetEnable(true);
				} else {
					uart0.Send_int(DualCar_UART::INT::i6, 0);
					led3.SetEnable(false);
				}
 */

#ifndef INC_BOARDID_H_
#define INC_BOARDID_H_

#ifndef newboard
#include "libbase/k60/cmsis/mk60f15.h"
#include <libbase/k60/mcg.h>
#include <libsc/system.h>
#include "libbase/k60/pit.h"
#include "libbase/k60/vectors.h"
#include "libbase/k60/gpio.h"

#include <cstddef>
#include <cstdint>
#include <functional>
#include <libsc/system.h>

class BoardID {
public:
	BoardID() :
			m_car(getGpiConfig(0)), m_board(getGpiConfig(1)) {
		m_isCar1 = m_car.Get();
		m_isBoardA = m_board.Get();
	}

	bool isCar1() {
		return m_isCar1;
	}

	bool isBoardA() {
		return m_isBoardA;
	}
private:
	const Pin::Name CAR_PIN = Pin::Name::kPte12;
	const Pin::Name BOARD_PIN = Pin::Name::kPte24;

	libbase::k60::Gpi m_car;
	libbase::k60::Gpi m_board;

	bool m_isCar1, m_isBoardA;

	libbase::k60::Gpi::Config getGpiConfig(uint8_t id) {
		libbase::k60::Gpi::Config config;
		config.config.set(Pin::Config::ConfigBit::kPullUp);
		config.interrupt = Pin::Config::Interrupt::kBoth;
		config.isr = [&] (libbase::k60::Gpi *gpi) {
			m_isCar1 = m_car.Get();
			m_isBoardA = m_board.Get();
		};
		if (id == 0) {
			config.pin = CAR_PIN;
		} else {
			config.pin = BOARD_PIN;
		}
		return config;
	}
};

#else

#ifndef INC_BOARDID_H_
#define INC_BOARDID_H_

class BoardID {
public:
	BoardID() :
			m_pin0(getGpiConfig(0)), m_pin1(getGpiConfig(1)), m_pin2(getGpiConfig(2)) {
		b_pin0 = m_pin0.Get();
		b_pin1 = m_pin1.Get();
		b_pin2 = m_pin2.Get();
	}

	uint8_t getCarID() {
		uint8_t id = 0;
		id += b_pin0 ? 1 : 0;
		id += b_pin1 ? 2 : 0;
		id += b_pin2 ? 4 : 0;
		return id;
	}

private:
	const Pin::Name J1 = Pin::Name::kPta19; // J1
	const Pin::Name J2 = Pin::Name::kPta11; // J2
	const Pin::Name J3 = Pin::Name::kPta10; // J3

	Gpi m_pin0, m_pin1, m_pin2;
	bool b_pin0, b_pin1, b_pin2;

	Gpi::Config getGpiConfig(uint8_t id) {
		Gpi::Config config;
		config.config.set(Pin::Config::ConfigBit::kPullUp);
		config.interrupt = Pin::Config::Interrupt::kBoth;
		if (id == 0) {
			config.pin = J1;
			config.isr = [&] (Gpi *gpi) {
				b_pin0 = m_pin0.Get();
			};
		} else if (id == 1) {
			config.pin = J2;
			config.isr = [&] (Gpi *gpi) {
				b_pin1 = m_pin1.Get();
			};
		} else if (id == 2) {
			config.pin = J3;
			config.isr = [&] (Gpi *gpi) {
				b_pin2 = m_pin2.Get();
			};
		}
		return config;
	}
};

#endif

#endif /* INC_BOARDID_H_ */
