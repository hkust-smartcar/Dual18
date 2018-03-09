/*
 * DualCar_UART_Config.h
 *
 *  Created on: 8 Mar 2018
 *      Author: JosephYim
 */

#ifndef INC_DUALCAR_UART_CONFIG_H_
#define INC_DUALCAR_UART_CONFIG_H_

#ifndef DEBUG
#define DEBUG
// no ack need
#endif

#ifndef SAVE_SPACE
#define SAVE_SPACE
// inline is used to speed up the program a little bit, but will
// occupy more space
// comment it to stop doing so
#endif

class DualCar_UART_Config {
public:
	/* 1)
	 * suggested naming convention, for easier future integration with other people
	 *
	 * YourName_VaribaleName
	 *
	 * e.g. Joseph_testNum
	 *
	 * 2) make sure both side has the same set of list
	 *
	 */

	enum class double_MailBox {
		// nah, max support for 32 terms so far, tell me if you want more la ^^
		test,
		MaxTerm // keep it as the last term
	};

	enum class uint8_t_MailBox {
		// nah, max support for 32 terms so far, tell me if you want more la ^^
		v0,
		v1,
		test,
		back,
		MaxTerm // keep it as the last term
	};

	enum class byte_MailBox {
		// nah, max support for 32 terms so far, tell me if you want more la ^^
		test,
		MaxTerm // keep it as the last term
	};

	enum class string_MailBox {
		// nah, max support for 32 terms so far, tell me if you want more la ^^
		test,
		MaxTerm // keep it as the last term
	};
};

#ifdef SAMPLE
// init
DualCar_UART uart(0, Uart::Config::BaudRate::k38400);

// init variables
// variable name not necessary as the mail box name la
// beware not to assign it for temporary local variable
// otherwise the program will crash
uart.DataCaller_uint8_t[(uint8_t) DualCar_UART::uint8_t_MailBox::test] = &testNum;
uart.DataCaller_uint8_t[(uint8_t) DualCar_UART::uint8_t_MailBox::v0] = &v0;
uart.DataCaller_uint8_t[(uint8_t) DualCar_UART::uint8_t_MailBox::v1] = &v1;
uart.DataCaller_double[(uint8_t) DualCar_UART::double_MailBox::test] = &test_d;
uart.DataCaller_byte[(uint8_t) DualCar_UART::byte_MailBox::test] = &test_b;

// auto send
uart.AutoSendWhenChanges_uint8_t.emplace_back(DualCar_UART::uint8_t_MailBox::test, 0);
uart.AutoSendWhenChanges_uint8_t.emplace_back(DualCar_UART::uint8_t_MailBox::v0, 0);
uart.AutoSendWhenChanges_uint8_t.emplace_back(DualCar_UART::uint8_t_MailBox::v1, 0);
uart.AutoSendWhenChanges_double.emplace_back(DualCar_UART::double_MailBox::test, 0);
uart.AutoSendWhenChanges_byte.emplace_back(DualCar_UART::byte_MailBox::test, 0);

// manual send
uart.Send_uint8_t(DualCar_UART::uint8_t_MailBox::test, testNum);

// important!
uart.RunEveryMS();

#endif

#endif /* INC_DUALCAR_UART_CONFIG_H_ */
