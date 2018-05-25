/*
 NO NEED TO EDIT THIS HEADER
 >> add(mailbox, variable pointer, isAutoSend)
 (if isAutoSend is used, the value will send to the client when it has a new value)
 void add(UINT8_T mailbox, uint8_t * ref, bool isAutoSend);
 void add(FLOAT mailbox, float * ref, bool isAutoSend);
 void add(DOUBLE mailbox, double * ref, bool isAutoSend);
 mailbox naming:
 UINT8_T: u0 to u30
 FLOAT: f0 to f30
 DOUBLE: d0 to d30
 Sample:
 DualCar_UART uart0;
 uart0.add(DualCar_UART::UINT8_T::u0, &test, false);
 *** run "uart0.parseValues();" once before in the while loop
 *** run "uart0.RunEveryMS();" once every ms
 suggest usage with Processing:
 set isAutoSend true when using OutputValueTile, Chart and Line
 set isAutoSend false when using InputIncDecTile
 */

#ifndef INC_DUALCAR_UART_H_
#define INC_DUALCAR_UART_H_

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

#define REPEAT_SEND_MS 100
#define LOCAL_BUFFER_MAX 8
#define BUFFER_SENT_MAX 30
// buffer size will determine the max len of the string
// keep it between 8 to 64

// no need debug la, not yet tested arrrr
#ifndef DEBUG
#define DEBUG
// no ack need
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

	enum class DOUBLE {
		// nah, max support for 32 terms so far, tell me if you want more la ^^
		d0,
		d1,
		d2,
		d3,
		d4,
		d5,
		d6,
		d7,
		d8,
		d9,
		d10,
		d11,
		d12,
		d13,
		d14,
		d15,
		d16,
		d17,
		d18,
		d19,
		d20,
		d21,
		d22,
		d23,
		d24,
		d25,
		d26,
		d27,
		d28,
		d29,
		d30,
		MaxTerm // keep it as the last term
	};

	enum class FLOAT {
		// nah, max support for 32 terms so far, tell me if you want more la ^^
		f0,
		f1,
		f2,
		f3,
		f4,
		f5,
		f6,
		f7,
		f8,
		f9,
		f10,
		f11,
		f12,
		f13,
		f14,
		f15,
		f16,
		f17,
		f18,
		f19,
		f20,
		f21,
		f22,
		f23,
		f24,
		f25,
		f26,
		f27,
		f28,
		f29,
		f30,
		MaxTerm // keep it as the last term
	};

	enum class UINT8_T {
		// nah, max support for 32 terms so far, tell me if you want more la ^^
		u0,
		u1,
		u2,
		u3,
		u4,
		u5,
		u6,
		u7,
		u8,
		u9,
		u10,
		u11,
		u12,
		u13,
		u14,
		u15,
		u16,
		u17,
		u18,
		u19,
		u20,
		u21,
		u22,
		u23,
		u24,
		u25,
		u26,
		u27,
		u28,
		u29,
		u30,
		MaxTerm
	// keep it as the last term
	};
};

class DualCar_UART: public DualCar_UART_Config {
public:
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

	DualCar_UART(const uint8_t &BT_id = 0, const Uart::Config::BaudRate &_BaudRate = Uart::Config::BaudRate::k38400) :
			bt(GetBluetoothConfig(BT_id, _BaudRate, [&](const Byte *data, const size_t size) {
				if (size == 1) {
				} // it is put here to remve the warning
					RxBuffer.push_back(*data);
					return true;
				})), SendCooling(0), needAck(true) {

		RxBuffer.clear();
		SendImmediate.clear();
		dataBuffer.clear();
		DataCaller_uint8_t.clear();
//		SendBuffer.clear();

		AutoSendWhenChanges_uint8_t.clear();
		AutoSendWhenChanges_double.clear();
		AutoSendWhenChanges_float.clear();

		for (uint8_t t = 0; t < LOCAL_BUFFER_MAX; t++) {
			dataBuffer.emplace_back(0);
		}

		for (uint8_t t = 0; t < static_cast<uint8_t>(UINT8_T::MaxTerm); t++) {
			DataCaller_uint8_t.emplace_back(nullptr);
		}

		for (uint8_t t = 0; t < static_cast<uint8_t>(DOUBLE::MaxTerm); t++) {
			DataCaller_double.emplace_back(nullptr);
		}

		for (uint8_t t = 0; t < static_cast<uint8_t>(FLOAT::MaxTerm); t++) {
			DataCaller_float.emplace_back(nullptr);
		}

#ifdef DEBUG
		needAck = false;
#endif
	}

	/*
	 * runs every ms to perform routine check
	 * 1) content of RxBuffer
	 * > check if the buffer size is >= 4
	 * > start code check
	 * > parity check
	 *
	 * !!! debugging
	 *
	 * 2) send the stuff in the send buffer
	 *
	 * @param
	 *
	 * @return
	 *
	 */
	uint32_t xd = 0;
	uint8_t BUFFER_SENT = 0;

	void RunEveryMS() {
		xd++;
		BUFFER_SENT = 4;

		RunEveryMS_StartTime = System::Time();
		SendCooling = SendCooling == 0 ? 0 : SendCooling - 1;

		if (xd % 50 == 0) {
			uint32_t timeNow = System::Time();
			SendWrapper(DATA_TYPE::SYSTEM, (uint8_t) SYSTEM_MSG::elpasedTime, static_cast<uint8_t>(timeNow),
					static_cast<uint8_t>(timeNow));
		}

		// send msg
		if (xd % 50 == 0) {
			if (AutoSendWhenChanges_uint8_t.size() != 0) {
				for (auto &temp : AutoSendWhenChanges_uint8_t) {
					uint8_t id = (uint8_t) temp.first;
					if ((DataCaller_uint8_t[id] != nullptr && BUFFER_SENT < BUFFER_SENT_MAX)
							&& (*DataCaller_uint8_t[id]) != temp.second) {
						Send_uint8_t(temp.first, *DataCaller_uint8_t[id]);
						temp.second = *DataCaller_uint8_t[id];
						BUFFER_SENT += 4;
					}
				}
			}

			if (AutoSendWhenChanges_double.size() != 0) {
				for (auto &temp : AutoSendWhenChanges_double) {
					uint8_t id = (uint8_t) temp.first;
					if ((DataCaller_double[id] != nullptr && BUFFER_SENT < BUFFER_SENT_MAX)
							&& (*DataCaller_double[id]) != temp.second) {
						Send_double(temp.first, *DataCaller_double[id]);
						temp.second = *DataCaller_double[id];
						BUFFER_SENT += 16;
					}
				}
			}

			if (AutoSendWhenChanges_float.size() != 0) {
				for (auto &temp : AutoSendWhenChanges_float) {
					uint8_t id = (uint8_t) temp.first;
					if ((DataCaller_float[id] != nullptr && BUFFER_SENT < BUFFER_SENT_MAX)
							&& (*DataCaller_float[id]) != temp.second) {
						Send_float(temp.first, *DataCaller_float[id]);
						temp.second = *DataCaller_float[id];
						BUFFER_SENT += 8;
					}
				}
			}
		}

		// handle incoming msg
		while (RxBuffer.size() >= 4) {
			if ((RxBuffer[0] == StartCode0 || RxBuffer[0] == StartCode1)
					&& isOdd(RxBuffer[0], RxBuffer[1], RxBuffer[2], RxBuffer[3])) {

				DATA_TYPE DataType = static_cast<DATA_TYPE>((RxBuffer[1] & 224) >> 5); // 0b1110 0000
				uint8_t MailBox_ = static_cast<uint8_t>(RxBuffer[1] & 31); // 0b00011111

				if (DataType == DATA_TYPE::BUFFER) {
					if (MailBox_ < (LOCAL_BUFFER_MAX / 2)) {
						dataBuffer[MailBox_ * 2] = static_cast<Byte>(RxBuffer[2]);
						dataBuffer[MailBox_ * 2 + 1] = static_cast<Byte>(RxBuffer[3]);
					}
				} else if (DataType == DATA_TYPE::UINT8_T) {
					// check if the mail box has been declared in the client side
					if (DataCaller_uint8_t.size() > MailBox_ && DataCaller_uint8_t[MailBox_] != nullptr) {
						*DataCaller_uint8_t[MailBox_] = static_cast<uint8_t>(RxBuffer[2]);
					}
				} else if (DataType == DATA_TYPE::DOUBLE) {
					if (DataCaller_double.size() > MailBox_ && DataCaller_double[MailBox_] != nullptr) {
						Byte* doublePtr = new Byte[8];

						for (uint8_t t = 0; t < 6; t++) {
							*(doublePtr + t) = dataBuffer[t];
						}
						*(doublePtr + 6) = RxBuffer[2];
						*(doublePtr + 7) = RxBuffer[3];
						memcpy(DataCaller_double[MailBox_], doublePtr, 8);

						delete[] (doublePtr);
						doublePtr = nullptr;
					}
				} else if (DataType == DATA_TYPE::FLOAT) {
					if (DataCaller_float.size() > MailBox_ && DataCaller_float[MailBox_] != nullptr) {
						Byte* floatPtr = new Byte[4];

						*(floatPtr + 0) = dataBuffer[0];
						*(floatPtr + 1) = dataBuffer[1];
						*(floatPtr + 2) = RxBuffer[2];
						*(floatPtr + 3) = RxBuffer[3];

						memcpy(DataCaller_float[MailBox_], floatPtr, 4);

						delete[] (floatPtr);
						floatPtr = nullptr;
					}
				} else if (DataType == DATA_TYPE::SYSTEM) {
					if (MailBox_ == (uint8_t) SYSTEM_MSG::ack) {
						// it is an ack
//						if (RxBuffer[2] == SendBuffer[0].COMMAND_CODE) {
//							// correct ack
//							SendCooling = 0;
//							SendBuffer.erase(SendBuffer.begin());
//						} else {
//							// wrong ack
//						}
					} else if (MailBox_ == (uint8_t) SYSTEM_MSG::sayHi) {
						// send all values to the request side
						if (AutoSendWhenChanges_uint8_t.size() != 0) {
							for (auto &temp : AutoSendWhenChanges_uint8_t) {
								uint8_t id = (uint8_t) temp.first;
								if (DataCaller_uint8_t[id] != nullptr) {
									Send_uint8_t(temp.first, *DataCaller_uint8_t[id]);
								}
							}
						}

						if (AutoSendWhenChanges_double.size() != 0) {
							for (auto &temp : AutoSendWhenChanges_double) {
								uint8_t id = (uint8_t) temp.first;
								if (DataCaller_double[id] != nullptr) {
									Send_double(temp.first, *DataCaller_double[id]);
								}
							}
						}
					}
				}

				// discard the first 4 buffer
				RxBuffer.erase(RxBuffer.begin());
				RxBuffer.erase(RxBuffer.begin());
				RxBuffer.erase(RxBuffer.begin());
				RxBuffer.erase(RxBuffer.begin());

			} else {
				// the pkg is wrong
				RxBuffer.erase(RxBuffer.begin());
			}
		}

		// 2) send the stuff in the send buffer
//		if (!SendBuffer.empty() && SendCooling == 0) {
//			Byte *pkgPtr = new Byte[4];
//
//			*pkgPtr = SendBuffer[0].START_CODE;
//			*(pkgPtr + 1) = SendBuffer[0].COMMAND_CODE;
//			*(pkgPtr + 2) = SendBuffer[0].Data_0;
//			*(pkgPtr + 3) = SendBuffer[0].Data_1;
//			bt.SendBuffer(pkgPtr, 4);
//
//			delete[] (pkgPtr);
//			pkgPtr = nullptr;
//
//			if (needAck == true) {
//				SendCooling = REPEAT_SEND_MS;
//			} else {
//				SendCooling = 0;
//				SendBuffer.erase(SendBuffer.begin());
//			}
//		}

		while (!SendImmediate.empty()) {
			Byte *pkgPtr = new Byte[4];
			*pkgPtr = SendImmediate[0].START_CODE;
			*(pkgPtr + 1) = SendImmediate[0].COMMAND_CODE;
			*(pkgPtr + 2) = SendImmediate[0].Data_0;
			*(pkgPtr + 3) = SendImmediate[0].Data_1;

			bt.SendBuffer(pkgPtr, 4);
			SendImmediate.erase(SendImmediate.begin());

			delete[] (pkgPtr);
			pkgPtr = nullptr;
		}
	}
	;

	/*
	 * parse values
	 * get values from master side upon a new connection
	 *
	 * @param
	 *
	 * @retrun
	 *
	 */

	void parseValues() {
		// parse all values from the other side
		// then send back my values
		SendWrapper(DATA_TYPE::SYSTEM, (uint8_t) SYSTEM_MSG::sayHi, 0, 0);

		if (AutoSendWhenChanges_uint8_t.size() != 0) {
			for (auto &temp : AutoSendWhenChanges_uint8_t) {
				uint8_t id = (uint8_t) temp.first;
				if (DataCaller_uint8_t[id] != nullptr) {
					Send_uint8_t(temp.first, *DataCaller_uint8_t[id]);
				}
			}
		}

		if (AutoSendWhenChanges_double.size() != 0) {
			for (auto &temp : AutoSendWhenChanges_double) {
				uint8_t id = (uint8_t) temp.first;
				if (DataCaller_double[id] != nullptr) {
					Send_double(temp.first, *DataCaller_double[id]);
				}
			}
		}

		if (AutoSendWhenChanges_float.size() != 0) {
			for (auto &temp : AutoSendWhenChanges_float) {
				uint8_t id = (uint8_t) temp.first;
				if (DataCaller_float[id] != nullptr) {
					Send_float(temp.first, *DataCaller_float[id]);
				}
			}
		}
	}

	/*
	 * init uint8_t
	 */

	void add(UINT8_T mailbox, uint8_t * ref, bool isAutoSend, uint8_t defaultValue) {
		add(mailbox, ref, isAutoSend);
		Send_uint8_t(mailbox, defaultValue);
	}

	void add(UINT8_T mailbox, uint8_t * ref, bool isAutoSend) {
		DataCaller_uint8_t[(uint8_t) mailbox] = ref;
		if (isAutoSend) {
			AutoSendWhenChanges_uint8_t.emplace_back(mailbox, 0);
		}
	}

	/*
	 * init float
	 */

	void add(FLOAT mailbox, float * ref, bool isAutoSend, float defaultValue) {
		add(mailbox, ref, isAutoSend);
		Send_float(mailbox, defaultValue);
	}

	void add(FLOAT mailbox, float * ref, bool isAutoSend) {
		DataCaller_float[(uint8_t) mailbox] = ref;
		if (isAutoSend) {
			AutoSendWhenChanges_float.emplace_back(mailbox, 0);
		}
	}

	/*
	 * init double
	 */

	void add(DOUBLE mailbox, double * ref, bool isAutoSend, double defaultValue) {
		add(mailbox, ref, isAutoSend);
		Send_double(mailbox, defaultValue);
	}

	void add(DOUBLE mailbox, double * ref, bool isAutoSend) {
		DataCaller_double[(uint8_t) mailbox] = ref;
		if (isAutoSend) {
			AutoSendWhenChanges_double.emplace_back(mailbox, 0);
		}
	}

private:
	struct PACKAGE {
		Byte START_CODE;
		Byte COMMAND_CODE;
		Byte Data_0;
		Byte Data_1;
	};

	enum class DATA_TYPE {
		BUFFER = 0, DOUBLE, UINT8_T, FLOAT, DIU, DIUDIU, DIUDIUDIU, SYSTEM = 7
	};

	// the attribute sayHi is for the client side to inquire for values
	// from the master side upon a new connection, e.g. restart
	enum class SYSTEM_MSG {
		ack = 0, sayHi, elpasedTime, lastTerm
	};

	JyMcuBt106 bt;
	std::vector<PACKAGE> SendImmediate;
//	std::vector<PACKAGE> SendBuffer;
	std::vector<Byte> RxBuffer;
	std::vector<Byte> dataBuffer;

	const Byte StartCode0 = 254; // 0b11111110
	const Byte StartCode1 = 255; // 0b11111111

	uint32_t RunEveryMS_StartTime;
	uint8_t SendCooling;
	bool needAck;

	std::vector<uint8_t*> DataCaller_uint8_t;
	std::vector<double*> DataCaller_double;
	std::vector<float*> DataCaller_float;

	std::vector<std::pair<UINT8_T, uint8_t>> AutoSendWhenChanges_uint8_t;
	std::vector<std::pair<DOUBLE, double>> AutoSendWhenChanges_double;
	std::vector<std::pair<FLOAT, float>> AutoSendWhenChanges_float;

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

	inline void Send_double(const DOUBLE &MailBox, const double &num) {

		Byte* doublePtr = new Byte[8];
		memcpy(doublePtr, &num, 8);

		SendWrapper(DATA_TYPE::BUFFER, 0, *(doublePtr + 0), *(doublePtr + 1));
		SendWrapper(DATA_TYPE::BUFFER, 1, *(doublePtr + 2), *(doublePtr + 3));
		SendWrapper(DATA_TYPE::BUFFER, 2, *(doublePtr + 4), *(doublePtr + 5));
		SendWrapper(DATA_TYPE::DOUBLE, (uint8_t) MailBox, *(doublePtr + 6), *(doublePtr + 7));

		delete[] (doublePtr);
		doublePtr = nullptr;
	}
	;

	/*
	 * send float
	 *
	 * @para
	 * MailBox: which MailBox will be called
	 * num: num to be sent
	 *
	 * @return
	 *
	 */

	inline void Send_float(const FLOAT &MailBox, const float &num) {
		Byte* floatPtr = new Byte[4];
		memcpy(floatPtr, &num, 4);

		SendWrapper(DATA_TYPE::BUFFER, 0, *(floatPtr + 0), *(floatPtr + 1));
		SendWrapper(DATA_TYPE::FLOAT, (uint8_t) MailBox, *(floatPtr + 2), *(floatPtr + 3));

		delete[] (floatPtr);
		floatPtr = nullptr;
	}
	;

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

	inline void Send_uint8_t(const UINT8_T &MailBox, const uint8_t &num) {
		SendWrapper(DATA_TYPE::UINT8_T, (uint8_t) MailBox, static_cast<Byte>(num), 0);
	}

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

	inline bool isOdd(const uint8_t &t0, const uint8_t &t1, const uint8_t &t2, const uint8_t &t3) {

		uint32_t t = 0;
		t ^= t0;
		t ^= t1 << 8;
		t ^= t2 << 16;
		t ^= t3 << 24;

		t ^= t >> 16;
		t ^= t >> 8;
		t ^= t >> 4;
		t ^= t >> 2;
		t ^= t >> 1;

		return (~t) & 1;
	}
	;

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

	inline void SendWrapper(const DATA_TYPE &DataType, const uint8_t &MailBox, const Byte &byte0, const Byte &byte1) {

		PACKAGE package;
		package.START_CODE = StartCode0;
		package.COMMAND_CODE = static_cast<Byte>(((static_cast<uint8_t>(DataType)) << 5)
				+ (static_cast<uint8_t>(MailBox)));
		package.Data_0 = byte0;
		package.Data_1 = byte1;

		if (!isOdd(package.START_CODE, package.COMMAND_CODE, package.Data_0, package.Data_1)) {
			package.START_CODE = StartCode1;
		}

		SendImmediate.emplace_back(package);

//		if (sendImmediateBool) {
//			SendImmediate.emplace_back(package);
//		} else {
//			SendBuffer.emplace_back(package);
//		}
	}

	/*
	 * bluetooth config
	 *
	 * @para
	 * id: device id
	 * _BaudRate: baudrate
	 * isr: listener function
	 *
	 * @return
	 * BT config
	 */

	inline JyMcuBt106::Config GetBluetoothConfig(const uint8_t &_id, const Uart::Config::BaudRate &_BaudRate,
			const std::function<bool(const Byte *data, const size_t size)> &isr) {
		JyMcuBt106::Config config;
		config.baud_rate = _BaudRate;
		config.id = _id;
		config.rx_isr = isr;
		config.tx_buf_size = 43;
		return config;
	}
	;

};

#endif /* INC_DUALCAR_UART_H_ */
