/*
 * FlashWrapper.h
 *
 *  Created on: 12 Jul 2018
 *      Author: JosephYim
 */

/*
sample

	uint8_t xd = 21;
	float diu = 2131;
	int pk = 31;
	bool on9 = false;

	FlashWrapper flash;
	flash.link_uint8_t(74, &xd);
	flash.link_float(44, &diu);
	flash.link_int(58, &pk);
	flash.link_bool(17, &on9);

	flash.writeFlash();
	flash.readFlash();

	flash.setBoardID(31);
	uint8_t boardID = flash.getBoardID();

*/

#ifndef INC_FLASHWRAPPER_H_
#define INC_FLASHWRAPPER_H_

#include "libbase/k60/flash.h"
#include <stdlib.h>

using libbase::k60::Flash;

/*
 * storage scheme
 * 0: flash wrapper ver
 * 1: sub version
 *
 * 2: mcu id
 * 3: mcu ver
 * 4: mcu Pos: master(1) / slave(0)
 *
 * 5: main board id
 *
 * 6 - 15: reserved
 *
 * 16 - 47: uint8_t
 * 48 - 175: int
 * 176 - 303: float
 * 304 - 335: bool
 */

#define FLASH_USAGE 336

class FlashWrapper {
public:
	FlashWrapper() :
			flash(getFlashConfig()) {

		// only read the config data
		Flash::FlashStatus status = flash.Read(FlashData, 16);
		if (status != Flash::FlashStatus::kSuccess) {
			assert(true);
		}

		flashWrapperVer = &FlashData[0];
		flashWrapperSubVer = &FlashData[1];
		mcuID = &FlashData[2];
		mcuVer = &FlashData[3];
		mcuPos = &FlashData[4];
		mainboardID = &FlashData[5];

		for (uint8_t i = 0; i < 32; i++) {
			uin8_tData[i] = nullptr;
			intData[i] = nullptr;
			floatData[i] = nullptr;
			boolData[i] = nullptr;
		}
	}

	inline void writeFlash() {
		// 0 to 15 are config
		for (uint8_t i = 0; i < 32; i++) {
			if (uin8_tData[i] != nullptr)
				memcpy(FlashData + 16 + i, uin8_tData[i], 1);
			if (intData[i] != nullptr)
				memcpy(FlashData + 48 + i * 4, intData[i], 4);
			if (floatData[i] != nullptr)
				memcpy(FlashData + 176 + i * 4, floatData[i], 4);
			if (boolData[i] != nullptr)
				memcpy(FlashData + 304 + i, boolData[i], 1);
		}
		Flash::FlashStatus status = flash.Write(FlashData, FLASH_USAGE);
		if (status != Flash::FlashStatus::kSuccess) {
			assert(true);
		}
	}

	inline void readFlash() {
		Flash::FlashStatus status = flash.Read(FlashData, FLASH_USAGE);
		if (status != Flash::FlashStatus::kSuccess) {
			assert(true);
		}
		for (uint8_t i = 0; i < 32; i++) {
			if (uin8_tData[i] != nullptr)
				memcpy(uin8_tData[i], FlashData + 16 + i, 1);
			if (intData[i] != nullptr)
				memcpy(intData[i], FlashData + 48 + i * 4, 4);
			if (floatData[i] != nullptr)
				memcpy(floatData[i], FlashData + 176 + i * 4, 4);
			if (boolData[i] != nullptr)
				memcpy(boolData[i], FlashData + 304 + i, 1);
		}
	}

	inline void link_uint8_t(const uint8_t &id, uint8_t * var) {
		if (id >= 32)
			assert(true);
		uin8_tData[id] = var;
	}

	inline void link_int(const uint8_t &id, int * var) {
		if (id >= 32)
			assert(true);
		intData[id] = var;
	}

	inline void link_float(const uint8_t &id, float * var) {
		if (id >= 32)
			assert(true);
		floatData[id] = var;
	}

	inline void link_bool(const uint8_t &id, bool * var) {
		if (id >= 32)
			assert(true);
		boolData[id] = var;
	}

	inline void setBoardID(uint8_t id) {
		*mainboardID = (Byte) id;
		saveConfig();
	}

	inline uint8_t getBoardID() {
		return (uint8_t) *mainboardID;
	}

	inline void setConfig() {
		*flashWrapperVer = (Byte) 0;
		*flashWrapperSubVer = (Byte) 0;
		*mcuID = (Byte) 0;
		*mcuVer = (Byte) 0;
		*mcuPos = (Byte) 0;
		*mainboardID = (Byte) 0;

		saveConfig();
	}

private:
	libbase::k60::Flash flash;
	Byte FlashData[FLASH_USAGE];
	Byte * flashWrapperVer;
	Byte * flashWrapperSubVer;
	Byte * mcuID;
	Byte * mcuVer;
	Byte * mcuPos;
	Byte * mainboardID;

	uint8_t * uin8_tData[32];
	int * intData[32];
	float * floatData[32];
	bool * boolData[32];

	inline void saveConfig() {
		Flash::FlashStatus status = flash.Write(FlashData, 16);
		if (status != Flash::FlashStatus::kSuccess) {
			assert(true);
		}
	}

	libbase::k60::Flash::Config getFlashConfig() {
		libbase::k60::Flash::Config config;
		// it seems there's no need to change
		return config;
	}
};

#endif /* INC_FLASHWRAPPER_H_ */
