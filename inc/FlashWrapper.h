/*
 * FlashWrapper.h
 *
 *  Created on: 12 Jul 2018
 *      Author: JosephYim
 */

#ifndef INC_FLASHWRAPPER_H_
#define INC_FLASHWRAPPER_H_

#include "libbase/k60/flash.h"

class FlashWrapper {
public:
	FlashWrapper() :
			flash(getFlashConfig()) {

	}

	bool readFloat(uint8_t floatNumber, std::vector<float> &f_, uint8_t intNumber, std::vector<float> &i_) {
		f_.clear();
		i_.clear();

		Byte * floatBytes;
		libbase::k60::Flash::FlashStatus status = flash.Read(floatBytes, floatNumber * 4);

		if (status == libbase::k60::Flash::FlashStatus::kSuccess) {
			float tempf;
			uint8_t t = 0;
			for (; t < floatNumber; t += 4) {
				memcpy(&tempf, (floatBytes + t), 4);
				f_.emplace_back(tempf);
			}
			int tempi;
			for (; t < intNumber; t += 4) {
				memcpy(&tempi, (floatBytes + t), 4);
				i_.emplace_back(tempi);
			}
			return true;
		} else {
			return false;
		}
	}

	bool writeFloat(std::vector<float> &f_, std::vector<int> &i_) {
		uint8_t floatNumber = f_.size();
		Byte * floatBytes;

		uint8_t i = 0;
		for (std::vector<float>::iterator it = f_.begin(); it != f_.end(); it++, i += 4) {
			// only a rather odd fix towards the problem
			// not elagant, admittedly
			float tf = *it; // dereference the iterator here
			memcpy((floatBytes + i), &tf, 4);
		}
		for (std::vector<int>::iterator it = i_.begin(); it != i_.end(); it++, i += 4) {
			// only a rather odd fix towards the problem
			// not elagant, admittedly
			int ti = *it; // dereference the iterator here
			memcpy((floatBytes + i), &ti, 4);
		}

		libbase::k60::Flash::FlashStatus status = flash.Write(floatBytes, floatNumber * 4);
		return (status == libbase::k60::Flash::FlashStatus::kSuccess);
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
