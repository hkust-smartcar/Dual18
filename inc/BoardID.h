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

	Gpi m_car;
	Gpi m_board;

	bool m_isCar1, m_isBoardA;

	Gpi::Config getGpiConfig(uint8_t id) {
		Gpi::Config config;
		config.config.set(Pin::Config::ConfigBit::kPullUp);
		config.interrupt = Pin::Config::Interrupt::kBoth;
		config.isr = [&] (Gpi *gpi) {
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

#endif /* INC_BOARDID_H_ */
