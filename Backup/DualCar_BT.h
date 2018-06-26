/*
 * DualCar_BT.h
 *
 *  Created on: 5 Mar 2018
 *      Author: JosephYim
 *
 * Odd parity
 *
 *  pkg structure:
 *  START_CODE
 *  COMMAND_CODE: xxx (data_type), xxxx x(MailBox)
 *
 * for uint8_t
 * START_CODE + COMMAND_CODE (MailBox) + DATA(num) + 0
 *
 * for double
 * START_CODE + COMMAND_CODE (Buffer 0 1) + Buffer 0 + Buffer 1
 * START_CODE + COMMAND_CODE (Buffer 2 3) + Buffer 2 + Buffer 3
 * START_CODE + COMMAND_CODE (Buffer 4 5) + Buffer 4 + Buffer 5
 * START_CODE + COMMAND_CODE (MailBox) + DATA 6 + DATA 7
 *
 * for byte
 * START_CODE + COMMAND_CODE (MailBox) + DATA(byte) + 0
 *
 * System Part (0-31)
 *
 * ack > (START_CODE)(111 + ?)(COMMAND_CODE received)(0)
 * RunEveryMS_Time > (START_CODE)(111 + ?)(time used)(0)
 * say hi > (START_CODE)(111 + ?)(TimeNow_0)(TimeNow_1)
 *
 */

#ifndef INC_DUALCAR_BT_H_
#define INC_DUALCAR_BT_H_

#ifndef DEBUG
#define DEBUG
#endif

#ifndef SAVE_SPACE
#define SAVE_SPACE
#endif

#include <cmath>
#include <vector>
#include <cassert>
#include <cstring>
#include <string>
#include <libbase/k60/mcg.h>
#include <libbase/k60/uart.h>
#include <libsc/system.h>

#include <libbase/k60/mcg.h>
#include <libsc/system.h>
#include "libsc/k60/jy_mcu_bt_106.h"

using libsc::System;
using libsc::k60::JyMcuBt106;
using libbase::k60::Uart;
using std::vector;

class DualCar_BT {
public:
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

	std::vector<uint8_t*> DataCaller_uint8_t;
	std::vector<double*> DataCaller_double;
	std::vector<Byte*> DataCaller_byte;

	/*
	 * init the class
	 *
	 * @param
	 * BT_id: bt id
	 * _BaudRate: baud rate
	 *
	 * @return
	 *
	 */

	DualCar_BT(const uint8_t &BT_id = 0, const Uart::Config::BaudRate &_BaudRate = Uart::Config::BaudRate::k38400);

	/*
	 * send double
	 *
	 * @para
	 * MailBox: which MailBox will be called
	 * num: num to be sent
	 *
	 * @return
	 *
	 */

	void Send_double(const double_MailBox &MailBox, const double &num);

	/*
	 * send uint8_t
	 *
	 * @para
	 * MailBox: which MailBox will be called
	 * num: num to be sent
	 *
	 * @return
	 *
	 */

	void Send_uint8_t(const uint8_t_MailBox &MailBox, const uint8_t &num);

	/*
	 * send byte
	 *
	 * @para
	 * MailBox: which MailBox will be called
	 * byte: byte to be sent
	 *
	 * @return
	 *
	 */

	void Send_byte(const byte_MailBox &MailBox, const Byte &byte);

	/*
	 * runs every ms to perform routine check
	 * 1) content of RxBuffer
	 * > check if the buffer size is >= 4
	 * > start code check
	 * > parity check
	 *
	 * 2) send the stuff in the send buffer
	 *
	 * @param
	 *
	 * @return
	 *
	 */

	void RunEveryMS();

//private:
	struct PACKAGE {
		Byte START_CODE;
		Byte COMMAND_CODE;
		Byte Data_0;
		Byte Data_1;
	};

	enum class DATA_TYPE {
		BUFFER = 0, DOUBLE, UINT8_T, BYTE, SYSTEM = 7
	};

	enum class SYSTEM_MSG {
		ack, sayHi, RunEveryMS_Time
	};

	JyMcuBt106 bt;
	std::vector<PACKAGE> SendImmediate;
	std::vector<PACKAGE> SendBuffer;
	std::vector<Byte> RxBuffer;
	std::vector<Byte> dataBuffer;

	const Byte StartCode0 = 254; // 0b11111110
	const Byte StartCode1 = 255; // 0b11111111

	uint16_t RunEveryMS_StartTime = 0;
	uint8_t SendCooling;
	bool needAck;

	/*
	 * parity checker
	 * ref https://stackoverflow.com/questions/21617970/how-to-check-if-value-has-even-parity-of-bits-or-odd
	 *
	 * @para
	 * t0
	 * t1
	 * t2
	 * t3
	 *
	 * @return
	 * return true if it's an odd number
	 * return false if it's an even number
	 */

	bool isOdd(const uint8_t &t0, const uint8_t &t1, const uint8_t &t2, const uint8_t &t3);

	/*
	 * SendWrapper
	 *
	 * @para
	 * DataType:
	 * MailBox:
	 * byte0:
	 * byte1:
	 *
	 * @return
	 *
	 */

	void SendWrapper(const DATA_TYPE &DataType, const uint8_t &MailBox, const Byte &byte0, const Byte &byte1,
			const bool &sendImmediateBool = false);

	/*
	 * bluetooth config
	 *
	 * @para
	 * MailBox: MailBox of the BT
	 * _BaudRate: baudrate
	 * isr: listener function
	 *
	 * @return
	 * BT config
	 */

	JyMcuBt106::Config GetBluetoothConfig(const uint8_t &_id, const Uart::Config::BaudRate &_BaudRate,
			const std::function<bool(const Byte *data, const size_t size)> &isr);
};

#endif /* INC_DUALCAR_BT_H_ */