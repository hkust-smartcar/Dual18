/*
 * FlashWrapper.h
 *
 *  Created on: 12 Jul 2018
 *      Author: JosephYim
 */

#ifndef INC_FLASHWRAPPER_H_
#define INC_FLASHWRAPPER_H_

#include "libbase/k60/flash.h"
#include <stdlib.h>

/*
 * Flash
 * 0: float size
 * 1: int size
 * 2+: data
 */

using libbase::k60::Flash;

class FlashWrapper {
public:
	FlashWrapper() :
			flash(getFlashConfig()) {

	}

	bool read(std::vector<float> &f_, std::vector<int> &i_) {
		f_.clear();
		i_.clear();

		// get the head of the flash
		Byte * Bytes = (Byte *) malloc(2);
		Flash::FlashStatus status = flash.Read(Bytes, 2);

		uint8_t floatNumber = Bytes[0];
		uint8_t intNumber = Bytes[1];
		uint8_t size = (floatNumber + intNumber) * 4 + 2;
		free(Bytes);

		if (status != Flash::FlashStatus::kSuccess) {
			return false;
		}

		// get data
		Bytes = (Byte *) malloc(size);
		status = flash.Read(Bytes, size);

		if (status != Flash::FlashStatus::kSuccess) {
			free(Bytes);
			return false;
		}

		uint8_t t = 2; // start from 2
		for (uint8_t i = 0; i < floatNumber; t += 4) {
			float tempf;
			memcpy(&tempf, (Bytes + t), 4);
			f_.emplace_back(tempf);
		}
		for (uint8_t i = 0; i < intNumber; t += 4) {
			int tempi;
			memcpy(&tempi, (Bytes + t), 4);
			i_.emplace_back(tempi);
		}

		free(Bytes);
		return true;
	}

	bool write(std::vector<float> &f_, std::vector<int> &i_) {
		uint8_t size = (f_.size() + i_.size()) * 4;
		Byte * Bytes = (Byte *) malloc(size + 2);

		Bytes[0] = f_.size();
		Bytes[1] = i_.size();

		uint8_t i = 2;
		for (std::vector<float>::iterator it = f_.begin(); it != f_.end(); it++, i += 4) {
			// only a rather odd fix towards the problem
			// not elagant, admittedly
			float tf = *it; // dereference the iterator here
			memcpy((Bytes + i), &tf, 4);
		}
		for (std::vector<int>::iterator it = i_.begin(); it != i_.end(); it++, i += 4) {
			// only a rather odd fix towards the problem
			// not elagant, admittedly
			int ti = *it; // dereference the iterator here
			memcpy((Bytes + i), &ti, 4);
		}
		free(Bytes);

		Flash::FlashStatus status = flash.Write(Bytes, size);
		return (status == Flash::FlashStatus::kSuccess);
	}
private:
	libbase::k60::Flash flash;

	libbase::k60::Flash::Config getFlashConfig() {
		libbase::k60::Flash::Config config;
		// it seems there's no need to change
		return config;
	}
};

#endif /* INC_FLASHWRAPPER_H_ */
