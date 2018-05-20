/*
 * DualCar_BT.cpp
 *
 *  Created on: 6 Mar 2018
 *      Author: JosephYim
 */

#include "DualCar_BT.h"

#define REPEAT_SEND_MS 100
#define LOCAL_BUFFER_MAX 8

DualCar_BT::DualCar_BT(const uint8_t &BT_id, const Uart::Config::BaudRate &_BaudRate) :
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
	SendBuffer.clear();

	for (uint8_t t = 0; t < LOCAL_BUFFER_MAX; t++) {
		dataBuffer.emplace_back(0);
	}

	for (uint8_t t = 0; t < static_cast<uint8_t>(uint8_t_MailBox::MaxTerm); t++) {
		DataCaller_uint8_t.emplace_back(nullptr);
	}

	for (uint8_t t = 0; t < static_cast<uint8_t>(double_MailBox::MaxTerm); t++) {
		DataCaller_double.emplace_back(nullptr);
	}

	for (uint8_t t = 0; t < static_cast<uint8_t>(byte_MailBox::MaxTerm); t++) {
		DataCaller_byte.emplace_back(nullptr);
	}

#ifdef DEBUG
	needAck = false;
#endif

}

void DualCar_BT::Send_double(const double_MailBox &MailBox, const double &num) {

	Byte* doublePtr = new Byte[8];
	memcpy(doublePtr, &num, 8);

	SendWrapper(DATA_TYPE::BUFFER, 0, *(doublePtr + 0), *(doublePtr + 1));
	SendWrapper(DATA_TYPE::BUFFER, 1, *(doublePtr + 2), *(doublePtr + 3));
	SendWrapper(DATA_TYPE::BUFFER, 2, *(doublePtr + 4), *(doublePtr + 5));
	SendWrapper(DATA_TYPE::DOUBLE, (uint8_t) MailBox, *(doublePtr + 6), *(doublePtr + 7));

	delete[] (doublePtr);
	doublePtr = nullptr;
}

void DualCar_BT::Send_uint8_t(const uint8_t_MailBox &MailBox, const uint8_t &num) {
	SendWrapper(DATA_TYPE::UINT8_T, (uint8_t) MailBox, static_cast<Byte>(num), 0);
}

void DualCar_BT::Send_byte(const byte_MailBox &MailBox, const Byte &byte) {
	SendWrapper(DATA_TYPE::BYTE, (uint8_t) MailBox, byte, 0);
}

void DualCar_BT::RunEveryMS() {
	RunEveryMS_StartTime = System::Time();
	SendCooling = SendCooling == 0 ? 0 : SendCooling - 1;

// 1) content of RxBuffer
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

				if (needAck) {
					SendWrapper(DATA_TYPE::SYSTEM, (uint8_t) SYSTEM_MSG::ack, RxBuffer[1], 0, true);
				}

			} else if (DataType == DATA_TYPE::UINT8_T) {
				// check if the mail box has been declared in the client side
				if (DataCaller_uint8_t.size() > MailBox_ && DataCaller_uint8_t[MailBox_] != nullptr) {
					*DataCaller_uint8_t[MailBox_] = static_cast<uint8_t>(RxBuffer[2]);
				}

				if (needAck) {
					SendWrapper(DATA_TYPE::SYSTEM, (uint8_t) SYSTEM_MSG::ack, RxBuffer[1], 0, true);
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

				if (needAck) {
					SendWrapper(DATA_TYPE::SYSTEM, (uint8_t) SYSTEM_MSG::ack, RxBuffer[1], 0, true);
				}

			} else if (DataType == DATA_TYPE::BYTE) {
				// check if the mail box has been declared in the client side
				if (DataCaller_byte.size() > MailBox_ && DataCaller_byte[MailBox_] != nullptr) {
					*DataCaller_byte[MailBox_] = static_cast<Byte>(RxBuffer[2]);
				}

				if (needAck) {
					SendWrapper(DATA_TYPE::SYSTEM, (uint8_t) SYSTEM_MSG::ack, RxBuffer[1], 0, true);
				}

			} else if (DataType == DATA_TYPE::SYSTEM) {
				if (MailBox_ == (uint8_t) SYSTEM_MSG::ack) {
					// it is an ack
					if (RxBuffer[2] == SendBuffer[0].COMMAND_CODE) {
						// correct ack
						SendCooling = 0;
						SendBuffer.erase(SendBuffer.begin());
					} else {
						// wrong ack
					}
				} else if (MailBox_ == (uint8_t) SYSTEM_MSG::sayHi) {

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
	if (!SendBuffer.empty() && SendCooling == 0) {
		Byte *pkgPtr = new Byte[4];

		*pkgPtr = SendBuffer[0].START_CODE;
		*(pkgPtr + 1) = SendBuffer[0].COMMAND_CODE;
		*(pkgPtr + 2) = SendBuffer[0].Data_0;
		*(pkgPtr + 3) = SendBuffer[0].Data_1;
		bt.SendBuffer(pkgPtr, 4);

		delete[] (pkgPtr);
		pkgPtr = nullptr;

		if (needAck == true) {
			SendCooling = REPEAT_SEND_MS;
		} else {
			SendCooling = 0;
			SendBuffer.erase(SendBuffer.begin());
		}
	}

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

	if (RunEveryMS_StartTime % 100 == 0) {
		uint16_t timeNow = System::Time();
		uint16_t diff = timeNow - RunEveryMS_StartTime;

		SendWrapper(DATA_TYPE::SYSTEM, (uint8_t) SYSTEM_MSG::sayHi, static_cast<Byte>(timeNow >> 8),
				static_cast<Byte>(timeNow & 255));
		SendWrapper(DATA_TYPE::SYSTEM, (uint8_t) SYSTEM_MSG::RunEveryMS_Time, static_cast<Byte>(diff), 0);
	}
}

#ifndef SAVE_SPACE
inline
#endif

bool DualCar_BT::isOdd(const uint8_t &t0, const uint8_t &t1, const uint8_t &t2, const uint8_t &t3) {

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

#ifndef SAVE_SPACE
inline
#endif

void DualCar_BT::SendWrapper(const DATA_TYPE &DataType, const uint8_t &MailBox, const Byte &byte0, const Byte &byte1,
		const bool &sendImmediateBool) {

	PACKAGE package;
	package.START_CODE = StartCode0;
	package.COMMAND_CODE =
			static_cast<Byte>(((static_cast<uint8_t>(DataType)) << 5) + (static_cast<uint8_t>(MailBox)));
	package.Data_0 = byte0;
	package.Data_1 = byte1;

	if (!isOdd(package.START_CODE, package.COMMAND_CODE, package.Data_0, package.Data_1)) {
		package.START_CODE = StartCode1;
	}

	if (sendImmediateBool) {
		SendImmediate.emplace_back(package);
	} else {
		SendBuffer.emplace_back(package);
	}
}

JyMcuBt106::Config DualCar_BT::GetBluetoothConfig(const uint8_t &_id, const Uart::Config::BaudRate &_BaudRate,
		const std::function<bool(const Byte *data, const size_t size)> &isr) {
	JyMcuBt106::Config config;
	config.baud_rate = _BaudRate;
	config.id = _id;
	config.rx_isr = isr;
	return config;
}
