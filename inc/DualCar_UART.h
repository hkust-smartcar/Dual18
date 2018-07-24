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

 * run "uart0.parseValues();" once before in the while loop
 * run "uart0.RunEveryMS();" once every ms

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

#define K60
//#define MORRIS_USING

#ifdef K60
#include "libbase/k60/cmsis/mk60f15.h"
#include <libbase/k60/mcg.h>
#include <libbase/k60/uart.h>
#include <libsc/system.h>
#include "libbase/k60/pin.h"
#include "libbase/k60/pinout.h"

#include <libbase/k60/mcg.h>
#include <libsc/system.h>
#include "libsc/k60/jy_mcu_bt_106.h"

using libsc::System;
using libsc::k60::JyMcuBt106;
using libbase::k60::Uart;

#ifdef MORRIS_USING
#include "corner.h"
#endif

#else

#define Byte uint8_t

#endif

using std::vector;

#define REPEAT_SEND_MS 100
#define LOCAL_BUFFER_MAX 8
#define BUFFER_SENT_MAX 80
#define CHECK_VALUE_PER 50
#define CONNECTION_LOST_TOLERANCE 3
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

	enum class HM10ACT {
		connectToA, connectToB, connectToC, connectToD, connectToE, setAsSlave
	};

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
		boolean,
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
		MaxTerm
	// keep it as the last term
	};

	enum class INT {
		// nah, max support for 32 terms so far, tell me if you want more la ^^
		i0,
		i1,
		i2,
		i3,
		i4,
		i5,
		i6,
		i7,
		i8,
		i9,
		i10,
		i11,
		i12,
		i13,
		i14,
		i15,
		i16,
		i17,
		i18,
		i19,
		i20,
		i21,
		i22,
		i23,
		i24,
		i25,
		i26,
		i27,
		i28,
		i29,
		i30,
		MaxTerm
	// keep it as the last term
	};

	enum class BOOLEAN {
		// nah, max support for 32 terms so far, tell me bf you want more la ^^
		b0,
		b1,
		b2,
		b3,
		b4,
		b5,
		b6,
		b7,
		b8,
		b9,
		b10,
		b11,
		b12,
		b13,
		b14,
		b15,
		b16,
		b17,
		b18,
		b19,
		b20,
		b21,
		b22,
		b23,
		b24,
		b25,
		b26,
		b27,
		b28,
		b29,
		b30,
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

	uint32_t getsth = 0;
	uint32_t sendsth = 0;

#ifdef K60
	DualCar_UART(const uint8_t &BT_id = 0, const Uart::Config::BaudRate &_BaudRate = Uart::Config::BaudRate::k115200) :
			bt(GetBluetoothConfig(BT_id, _BaudRate, [&](const Byte *data, const size_t size) {
				RxBuffer.emplace_back(*data);
				getsth++;
				return true;
			})), SendCooling(0) {
#else
		DualCar_UART(const uint8_t &BT_id = 0) :
		SendCooling(0), needAck(true) {
#endif
		RxBuffer.clear();
		SendImmediate.clear();
		dataBuffer.clear();
		DataCaller_uint8_t.clear();
//		SendBuffer.clear();

		AutoSendWhenChanges_uint8_t.clear();
		AutoSendWhenChanges_double.clear();
		AutoSendWhenChanges_float.clear();
		AutoSendWhenChanges_int.clear();
		AutoSendWhenChanges_bool.clear();

		for (uint8_t t = 0; t < LOCAL_BUFFER_MAX; t++) {
#ifdef K60
			dataBuffer.emplace_back(0);
#else
			dataBuffer.push_back(0);
#endif
		}

		for (uint8_t t = 0; t < static_cast<uint8_t>(UINT8_T::MaxTerm); t++) {
#ifdef K60
			DataCaller_uint8_t.emplace_back(nullptr);
#else
			DataCaller_uint8_t.push_back(nullptr);
#endif
		}

		DataCaller_uint8_t[(uint8_t) UINT8_T::boolean] = &booleanT;

		for (uint8_t t = 0; t < static_cast<uint8_t>(DOUBLE::MaxTerm); t++) {
#ifdef K60
			DataCaller_double.emplace_back(nullptr);
#else
			DataCaller_double.push_back(nullptr);
#endif
		}

		for (uint8_t t = 0; t < static_cast<uint8_t>(FLOAT::MaxTerm); t++) {
#ifdef K60
			DataCaller_float.emplace_back(nullptr);
#else
			DataCaller_float.push_back(nullptr);
#endif
		}

		for (uint8_t t = 0; t < static_cast<uint8_t>(INT::MaxTerm); t++) {
#ifdef K60
			DataCaller_int.emplace_back(nullptr);
#else
			DataCaller_int.push_back(nullptr);
#endif
		}

		for (uint8_t t = 0; t < static_cast<uint8_t>(BOOLEAN::MaxTerm); t++) {
#ifdef K60
			DataCaller_bool.emplace_back(nullptr);
#else
			DataCaller_bool.push_back(nullptr);
#endif
		}
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
	uint32_t lastSent = 0;
	uint8_t booleanT;

	void RunEveryMS() {
		xd++;
		BUFFER_SENT = 4;

#ifdef K60
		RunEveryMS_StartTime = System::Time();
#else
		RunEveryMS_StartTime = millis();
#endif
		SendCooling = SendCooling == 0 ? 0 : SendCooling - 1;

		if ((RunEveryMS_StartTime - lastSent) >= 700) {
			lastSent = RunEveryMS_StartTime;
			uint32_t timeNow = RunEveryMS_StartTime;

			timeNow /= 1000;
			SendWrapper(DATA_TYPE::SYSTEM, (uint8_t) SYSTEM_MSG::elpasedTime, static_cast<uint8_t>(timeNow >> 8),
					static_cast<uint8_t>(timeNow));

			isConnectedTimeOut = isConnectedTimeOut > 0 ? isConnectedTimeOut - 1 : 0;
		}

		// send msg
		if (xd % CHECK_VALUE_PER == 0) {
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

			if (AutoSendWhenChanges_bool.size() != 0) {
				for (auto &temp : AutoSendWhenChanges_bool) {
					uint8_t id = (uint8_t) temp.first;
					if ((DataCaller_bool[id] != nullptr) && (*DataCaller_bool[id]) != temp.second) {
						Send_bool(temp.first, *DataCaller_bool[id]);
						temp.second = *DataCaller_bool[id];
						BUFFER_SENT += 4;
					}
				}
			}

			if (AutoSendWhenChanges_int.size() != 0) {
				for (auto &temp : AutoSendWhenChanges_int) {
					uint8_t id = (uint8_t) temp.first;
					if ((DataCaller_int[id] != nullptr && BUFFER_SENT < BUFFER_SENT_MAX)
							&& (*DataCaller_int[id]) != temp.second) {
						Send_int(temp.first, *DataCaller_int[id]);
						temp.second = *DataCaller_int[id];
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
					if (DataCaller_uint8_t.size() > MailBox_ && DataCaller_uint8_t[MailBox_] != nullptr) {

						if (MailBox_ == (uint8_t) UINT8_T::boolean
								&& (DataCaller_bool.size() > (uint8_t) RxBuffer[2]
										&& DataCaller_bool[(uint8_t) RxBuffer[2]] != nullptr)) {
							*DataCaller_bool[(uint8_t) RxBuffer[2]] = ((uint8_t) RxBuffer[3] >= 1);
						} else {
							*DataCaller_uint8_t[MailBox_] = static_cast<uint8_t>(RxBuffer[2]);
						}
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
				} else if (DataType == DATA_TYPE::INT) {
					if (DataCaller_int.size() > MailBox_ && DataCaller_int[MailBox_] != nullptr) {
						Byte* intPtr = new Byte[4];

						*(intPtr + 0) = dataBuffer[0];
						*(intPtr + 1) = dataBuffer[1];
						*(intPtr + 2) = RxBuffer[2];
						*(intPtr + 3) = RxBuffer[3];

						memcpy(DataCaller_int[MailBox_], intPtr, 4);

						delete[] (intPtr);
						intPtr = nullptr;
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
						// to be completed
					} else if (MailBox_ == (uint8_t) SYSTEM_MSG::elpasedTime) {
						receivedElpasedTime = (RxBuffer[2] << 8) + RxBuffer[3];
						isConnectedTimeOut = CONNECTION_LOST_TOLERANCE;

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

		while (!SendImmediate.empty()) {
			Byte *pkgPtr = new Byte[4];
			*pkgPtr = SendImmediate[0].START_CODE;
			*(pkgPtr + 1) = SendImmediate[0].COMMAND_CODE;
			*(pkgPtr + 2) = SendImmediate[0].Data_0;
			*(pkgPtr + 3) = SendImmediate[0].Data_1;

			SendImmediate.erase(SendImmediate.begin());

			sendsth++;

#ifdef K60
			bt.SendBuffer(pkgPtr, 4);
#else
			Serial.write(pkgPtr, 4);
#endif

			delete[] (pkgPtr);
			pkgPtr = nullptr;
		}
	}

	bool isConnected() {
		return (isConnectedTimeOut > 0);
	}

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
#ifdef K60
			AutoSendWhenChanges_uint8_t.emplace_back(mailbox, 0);
#else
			AutoSendWhenChanges_uint8_t.push_back(std::pair<UINT8_T, uint8_t> {
						mailbox, 0});
#endif
		}
	}

	/*
	 * init bool
	 */

	void add(BOOLEAN mailbox, bool * ref, bool isAutoSend, bool defaultValue) {
		add(mailbox, ref, isAutoSend);
		Send_bool(mailbox, defaultValue);
	}

	void add(BOOLEAN mailbox, bool * ref, bool isAutoSend) {
		DataCaller_bool[(uint8_t) mailbox] = ref;
		if (isAutoSend) {
#ifdef K60
			AutoSendWhenChanges_bool.emplace_back(mailbox, 0);
#else
			AutoSendWhenChanges_bool.push_back(std::pair<BOOLEAN, bool> {
						mailbox, 0});
#endif
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
#ifdef K60
			AutoSendWhenChanges_float.emplace_back(mailbox, 0);
#else
			AutoSendWhenChanges_float.push_back(std::pair<FLOAT, float> {
						mailbox, 0});
#endif
		}
	}

	/*
	 * init int
	 */

	void add(INT mailbox, int32_t * ref, bool isAutoSend, int32_t defaultValue) {
		add(mailbox, ref, isAutoSend);
		Send_int(mailbox, defaultValue);
	}

	void add(INT mailbox, int32_t * ref, bool isAutoSend) {
		DataCaller_int[(uint8_t) mailbox] = ref;
		if (isAutoSend) {
#ifdef K60
			AutoSendWhenChanges_int.emplace_back(mailbox, 0);
#else
			AutoSendWhenChanges_int.push_back(std::pair<INT, int32_t> {mailbox,
						0});
#endif
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
#ifdef K60
			AutoSendWhenChanges_double.emplace_back(mailbox, 0);
#else
			AutoSendWhenChanges_double.push_back(std::pair<DOUBLE, double> {
						mailbox, 0});
#endif
		}
	}

	/*
	 * send corners
	 *
	 * @para
	 * vect: the corner vector to be sent
	 *
	 * @return
	 *
	 */

//#ifndef Corner
//	class Corner {
//	public:
//		Corner() :
//				percentage(0), x(0), y(0) {
//		}
//		Corner(int m_x, int m_y, float m_percentage) :
//				percentage(m_percentage), x(m_x), y(m_y) {
//		}
//		void set_percentage(float m_percentage) {
//			percentage = m_percentage;
//		}
//		void set_xcoord(int m_x) {
//			x = m_x;
//		}
//		void set_ycoord(int m_y) {
//			y = m_y;
//		}
//
//		float get_percentage() {
//			return percentage;
//		}
//		int get_xcoord() {
//			return x;
//		}
//		int get_ycoord() {
//			return y;
//		}
//
//	private:
//		float percentage;
//		int x;
//		int y;
//	};
//#endif
#ifdef MORRIS_USING
	inline void Send_corners(
			std::vector<Corner> &vect) {

		uint8_t vectorSize = vect.size();
		vectorSize = vectorSize > 32 ? 32 : vectorSize;

		SendWrapper(DATA_TYPE::SYSTEM, (uint8_t) SYSTEM_MSG::vector_StartSend,
				(uint8_t) vect[0].get_xcoord(), (uint8_t) vect[0].get_ycoord());

		for (uint8_t i = 1; i < vectorSize; i++) {
			SendWrapper(DATA_TYPE::MORRIS_VECTOR, i-1, (uint8_t) vect[i].get_xcoord(),
					(uint8_t) vect[i].get_ycoord());
		}

		SendWrapper(DATA_TYPE::SYSTEM, (uint8_t) SYSTEM_MSG::vector_EndSend,
				vectorSize, 0);
	}
#endif

	inline void Send_log(std::string s_) {

		uint8_t stringSize = s_.size();
		if (stringSize % 2 == 1) {
			s_ += " ";
			stringSize = s_.size();
		}
		stringSize = stringSize > 64 ? 64 : stringSize;

		SendWrapper(DATA_TYPE::SYSTEM, (uint8_t) SYSTEM_MSG::log_StartSend, (uint8_t) s_[0], (uint8_t) s_[1]);

		for (uint8_t i = 2; i < stringSize; i += 2) {
			SendWrapper(DATA_TYPE::LOG, i / 2 - 1, (uint8_t) s_[i], (uint8_t) s_[i + 1]);
		}

		SendWrapper(DATA_TYPE::SYSTEM, (uint8_t) SYSTEM_MSG::log_EndSend, stringSize, 0);
	}

#ifdef DEBUG_PHASE
	inline void Send_camera(const Byte * b, const uint8_t &w, const uint8_t &h) {

		SendWrapper(DATA_TYPE::SYSTEM, (uint8_t) SYSTEM_MSG::camera_StartSend,
				w, h);

		int len = w * h / 8 / 2;
		for (int i = 0; i < len; i += 2) {
			SendWrapper(DATA_TYPE::CAMERA_IMG, (uint8_t) (i>>1), *(b + i), *(b + i + 1));
		}

		SendWrapper(DATA_TYPE::SYSTEM, (uint8_t) SYSTEM_MSG::camera_EndSend, 0,
				0);
	}
#endif

//private:
	struct PACKAGE {
		Byte START_CODE;
		Byte COMMAND_CODE;
		Byte Data_0;
		Byte Data_1;
	};

	enum class DATA_TYPE {
//		BUFFER = 0, DOUBLE, UINT8_T, FLOAT, INT, MORRIS_VECTOR, CAMERA_IMG, SYSTEM = 7, BOOLEAN
		BUFFER = 0,
		DOUBLE,
		UINT8_T,
		FLOAT,
		INT,
		LOG,
		CAMERA_IMG,
		SYSTEM = 7,
		BOOLEAN
	};

	// the attribute sayHi is for the client side to inquire for values
	// from the master side upon a new connection, e.g. restart
	enum class SYSTEM_MSG {
		ack = 0,
		sayHi,
		elpasedTime,
		vector_StartSend,
		vector_EndSend,
		camera_StartSend,
		camera_EndSend,
		log_StartSend,
		log_EndSend,
		lastTerm
	};

#ifdef K60
	JyMcuBt106 bt;
#endif

	std::vector<PACKAGE> SendImmediate;
//	std::vector<PACKAGE> SendBuffer;
	std::vector<Byte> RxBuffer;
	std::vector<Byte> dataBuffer;

	const Byte StartCode0 = 254; // 0b11111110
	const Byte StartCode1 = 255; // 0b11111111

	uint32_t RunEveryMS_StartTime;
	uint8_t SendCooling;
	uint16_t receivedElpasedTime = 0;
	uint8_t isConnectedTimeOut = 0;

	std::vector<uint8_t*> DataCaller_uint8_t;
	std::vector<double*> DataCaller_double;
	std::vector<float*> DataCaller_float;
	std::vector<int32_t*> DataCaller_int;
	std::vector<bool*> DataCaller_bool;

	std::vector<std::pair<UINT8_T, uint8_t>> AutoSendWhenChanges_uint8_t;
	std::vector<std::pair<DOUBLE, double>> AutoSendWhenChanges_double;
	std::vector<std::pair<FLOAT, float>> AutoSendWhenChanges_float;
	std::vector<std::pair<INT, int32_t>> AutoSendWhenChanges_int;
	std::vector<std::pair<DualCar_UART_Config::BOOLEAN, bool>> AutoSendWhenChanges_bool;

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

	/*
	 * send int
	 *
	 * @para
	 * MailBox: which MailBox will be called
	 * num: num to be sent
	 *
	 * @return
	 *
	 */

	inline void Send_int(const INT &MailBox, const int32_t &num) {
//	inline void Send_int(const INT &MailBox, int32_t num) {
		Byte* intPtr = new Byte[4];
		memcpy(intPtr, &num, 4);

		SendWrapper(DATA_TYPE::BUFFER, 0, *(intPtr + 0), *(intPtr + 1));
		SendWrapper(DATA_TYPE::INT, (uint8_t) MailBox, *(intPtr + 2), *(intPtr + 3));

		delete[] (intPtr);
		intPtr = nullptr;
	}

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

	inline void Send_uint8_t(const UINT8_T &MailBox, const uint8_t &num, const Byte &num0 = 0) {
		SendWrapper(DATA_TYPE::UINT8_T, (uint8_t) MailBox, static_cast<Byte>(num), num0);
	}

	/*
	 * send bool
	 *
	 * @para
	 * MailBox: which MailBox will be called
	 * num: num to be sent
	 *
	 * @return
	 *
	 */

	inline void Send_bool(const DualCar_UART_Config::BOOLEAN &MailBox, const bool &num) {
		Send_uint8_t(UINT8_T::boolean, (uint8_t) MailBox, (Byte) num ? 0b1 : 0b0);
//		Send_uint8_t(UINT8_T::boolean, (uint8_t) MailBox, (Byte) num ? 0b1 : 0b0);
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

	inline bool isOdd(uint8_t t0, uint8_t t1, uint8_t t2, uint8_t t3) {

		uint32_t t = 0;
//		t ^= t0;
//		t ^= t1 << 8;
//		t ^= t2 << 16;
//		t ^= t3 << 24;
//
//		t ^= t >> 16;
//		t ^= t >> 8;
//		t ^= t >> 4;
//		t ^= t >> 2;
//		t ^= t >> 1;
//
//		return (~t) & 1;

		for (int i = 0; i < 8; i++) {
			if ((t0 & 1) == 1) {
				t++;
			}
			if ((t1 & 1) == 1) {
				t++;
			}
			if ((t2 & 1) == 1) {
				t++;
			}
			if ((t3 & 1) == 1) {
				t++;
			}

			t0 = (Byte) (t0 >> 1);
			t1 = (Byte) (t1 >> 1);
			t2 = (Byte) (t2 >> 1);
			t3 = (Byte) (t3 >> 1);
		}

		return (t % 2 == 1);

//		return true;
	}

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

#ifdef K60
		SendImmediate.emplace_back(package);
#else
		SendImmediate.push_back(package);
#endif
	}

	void HM10Func(HM10ACT act) {
		switch (act) {
			case HM10ACT::connectToA: {
				HM10sendMsg("AT+ROLE1");
				System::DelayMs(100);
				HM10sendMsg("AT+COND43639DAFD54");
				System::DelayMs(100);
			}
				break;
			case HM10ACT::connectToB: {
				HM10sendMsg("AT+ROLE1");
				System::DelayMs(100);
				HM10sendMsg("AT+CON341513DB76D6");
				System::DelayMs(100);
			}
				break;
			case HM10ACT::connectToC: {
				HM10sendMsg("AT+ROLE1");
				System::DelayMs(100);
				HM10sendMsg("AT+CON34151387B3B2");
				System::DelayMs(100);
			}
				break;
			case HM10ACT::connectToD: {
				HM10sendMsg("AT+ROLE1");
				System::DelayMs(100);
				HM10sendMsg("AT+CON341513879A13");
				System::DelayMs(100);
			}
			case HM10ACT::connectToE: {
				HM10sendMsg("AT+ROLE1");
				System::DelayMs(100);
				HM10sendMsg("AT+COND43639DAF9EE");
				System::DelayMs(100);
			}
				break;
			case HM10ACT::setAsSlave: {
				HM10sendMsg("AT+ROLE0");
				System::DelayMs(100);
			}
				break;
		}

		HM10sendMsg("RESET");
		System::DelayMs(100);
	}

	void HM10sendMsg(char * msg) {
		if (RxBuffer.size() != 0)
			RxBuffer.emplace_back('\n');
		bt.SendBuffer((Byte *) msg, strlen(msg));
	}

#ifdef K60
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
		return config;
	}
#endif

};

#endif /* INC_DUALCAR_UART_H_ */
