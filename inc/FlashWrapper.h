/*
 * FlashWrapper.h
 *
 *  Created on: 12 Jul 2018
 *      Author: JosephYim
 */

#ifndef INC_FLASHWRAPPER_H_
#define INC_FLASHWRAPPER_H_

#include "libbase/k60/flash.h"
#include "libbase/k60/cmsis/mk60f15.h"
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
// 336 > 0x150

class FlashWrapper {
public:
	int iMCUVer;
	int imcuPos;
	int imainboardID;
	int actID = 0;;

	FlashWrapper() :
			flash(getFlashConfig()) {

		// only read the config data
		Flash::FlashStatus status = flash.Read(FlashData, 16);
		if (status != Flash::FlashStatus::kSuccess) {
			assert(false);
		}

		MCUVer = &FlashData[3];
		mcuPos = &FlashData[4];
		mainboardID = &FlashData[5];

		iMCUVer = (uint8_t) *MCUVer;
		imcuPos = (uint8_t) *mcuPos;
		imainboardID = (uint8_t) *mainboardID;

		for (uint8_t i = 0; i < 32; i++) {
			uin8_tData[i] = nullptr;
			intData[i] = nullptr;
			floatData[i] = nullptr;
			boolData[i] = nullptr;
		}

		bool sizeCheck = (sizeof(uint8_t) != 1) || (sizeof(int) != 4) || (sizeof(float) != 4)
				|| (sizeof(bool) != 1);
		if (sizeCheck)
			assert(false);

		actID++;
	}

	void writeFlash() {
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

		__disable_irq();
		libsc::System::DelayMs(100);
		Flash::FlashStatus status = flash.Write(FlashData, FLASH_USAGE);
		libsc::System::DelayMs(500);
		if (status != Flash::FlashStatus::kSuccess) {
			assert(false);
		}

		__enable_irq();
		actID++;
	}

	void readFlash() {
		__disable_irq();
		libsc::System::DelayMs(100);
		Flash::FlashStatus status = flash.Read(FlashData, FLASH_USAGE);
		libsc::System::DelayMs(100);
		if (status != Flash::FlashStatus::kSuccess) {
			assert(false);
		}
		__enable_irq();
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
		actID++;
	}

	inline void link_uint8_t(const uint8_t &id, uint8_t * var) {
		if (id >= 32)
			assert(false);
		uin8_tData[id] = var;
	}

	inline void link_int(const uint8_t &id, int * var) {
		if (id >= 32)
			assert(false);
		intData[id] = var;
	}

	inline void link_float(const uint8_t &id, float * var) {
		if (id >= 32)
			assert(false);
		floatData[id] = var;
	}

	inline void link_bool(const uint8_t &id, bool * var) {
		if (id >= 32)
			assert(false);
		boolData[id] = var;
	}

// mainboardID
	inline void setBoardID(uint8_t id) {
		*mainboardID = (Byte) id;
		saveConfig();
	}

	inline uint8_t getBoardID() {
		return (uint8_t) *mainboardID;
	}

// MCUVer
	inline void setMCUVer(uint8_t id) {
		*MCUVer = (Byte) id;
		saveConfig();
	}

	inline uint8_t getMCUVer() {
		return (uint8_t) *MCUVer;
	}

// mcuPos
// 1 for master, 0 for slave
	inline void setAsMaster() {
		*mcuPos = 1;
	}

	inline void setAsSlave() {
		*mcuPos = 0;
	}

	inline bool isMaster() {
		return (*mcuPos == 1);
	}

	void saveConfigFromMenu() {

		*MCUVer = (Byte) (uint8_t) iMCUVer;
		*mcuPos = (Byte) (uint8_t) imcuPos;
		*mainboardID = (Byte) (uint8_t) imainboardID;

		saveConfig();
	}

// config
	inline uint8_t getLEDConfig(uint8_t id) {
		if (*MCUVer == 1) {
			return id;
		} else if (*MCUVer == 2) {
			if (id == 0)
				return 2;
			if (id == 1)
				return 0;
			if (id == 2)
				return 1;
		}
		return 0;
	}

	inline uint8_t getCameraConfig() {
		if (*MCUVer == 1) {
			return 0;
		} else if (*MCUVer == 2) {
			return 1;
		}
	}

	inline uint8_t getUARTConfig(uint8_t id) {
		if (id == 2) {
			if (*MCUVer == 1) {
				return 2;
			} else if (*MCUVer == 2) {
				return 3;
			}
		} else {
			return id;
		}
		return 0;
	}

	inline uint8_t getBTtoComputerID() {
		if (*MCUVer == 1) {
			return 2;
		} else if (*MCUVer == 2) {
			return 3;
		}
	}

private:
	libbase::k60::Flash flash;
	Byte * FlashData = new Byte[FLASH_USAGE];
	Byte * flashWrapperVer = nullptr;
	Byte * flashWrapperSubVer = nullptr;
	Byte * mcuID = nullptr;
	Byte * MCUVer = nullptr;
	Byte * mcuPos = nullptr;
	Byte * mainboardID = nullptr;

	uint8_t * uin8_tData[32];
	int * intData[32];
	float * floatData[32];
	bool * boolData[32];

	inline void saveConfig() {
		__disable_irq();
		Flash::FlashStatus status = flash.Write(FlashData, 16);
		libsc::System::DelayMs(100);
		if (status != Flash::FlashStatus::kSuccess) {
			assert(false);
		}
		__enable_irq();
		actID++;
	}

	libbase::k60::Flash::Config getFlashConfig() {
		libbase::k60::Flash::Config config;
		// it seems there's no need to change
		return config;
	}
};

#endif /* INC_FLASHWRAPPER_H_ */
