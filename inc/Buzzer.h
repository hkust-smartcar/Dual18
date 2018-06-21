/*
 * Buzzer.h
 *
 *  Created on: 17 Jun 2018
 *      Author: JosephYim
 */

#ifndef INC_BUZZER_H_
#define INC_BUZZER_H_


#include <libutil/notes.h>
#include "libsc/simple_buzzer.h"
#include "libsc/passive_buzzer.h"
#include <math.h>
#include <vector>
using std::vector;
class musicPlayer {
public:
	enum note {
		none, c, cs, d, ds, e, f, fs, g, gs, a, as, b
	};

	struct node {
		note n;
		uint8_t octal;
		float duration;
		node(note _n, uint8_t _octal, float _d) :
				n(_n), octal(_octal), duration(_d) {
		}
	};

	musicPlayer() :
			buzzzer(getBuzzerConfig()) {
	}

	PassiveBuzzer buzzzer;
	float multiplier = 60 * 1000 / 150;

	uint8_t t = 4;
	std::vector<musicPlayer::node> NokiaSound = { { e, t, 0.5 }, { d, t, 0.5 }, { fs, t, 1 }, { gs, t, 1 }, { c, t,
			0.5 }, { b, t, 0.5 }, { d, t, 1 }, { e, t, 1 }, { b, t, 0.5 }, { a, t, 0.5 }, { cs, t, 1 },
			{ e, t, 1 }, { a, t, 3 } };

	void playSong(std::vector<node> song) {
		for (auto &t : song) {
			buzzzer.SetNote(getFreq(t.n, t.octal));
			System::DelayMs(t.duration * multiplier);
			buzzzer.SetBeep(true);
		}
		buzzzer.SetBeep(false);
	}

	const float c_freq = 4186.01;
	const float cs_freq = 4434.92;
	const float d_freq = 4698.63;
	const float ds_freq = 4978.03;
	const float e_freq = 5274.04;
	const float f_freq = 5587.65;
	const float fs_freq = 5919.91;
	const float g_freq = 6271.93;
	const float gs_freq = 6644.88;
	const float a_freq = 7040.00;
	const float as_freq = 7458.62;
	const float b_freq = 7901.13;

	float getFreq(note &n, uint8_t &octal) {
		switch (n) {
			case note::c:
				return (c_freq / pow(2, 8 - octal));
				break;
			case note::cs:
				return (cs_freq / pow(2, 8 - octal));
				break;
			case note::d:
				return (d_freq / pow(2, 8 - octal));
				break;
			case note::ds:
				return (ds_freq / pow(2, 8 - octal));
				break;
			case note::e:
				return (e_freq / pow(2, 8 - octal));
				break;
			case note::f:
				return (f_freq / pow(2, 8 - octal));
				break;
			case note::fs:
				return (fs_freq / pow(2, 8 - octal));
				break;
			case note::g:
				return (g_freq / pow(2, 8 - octal));
				break;
			case note::gs:
				return (gs_freq / pow(2, 8 - octal));
				break;
			case note::a:
				return (a_freq / pow(2, 8 - octal));
				break;
			case note::as:
				return (as_freq / pow(2, 8 - octal));
				break;
			case note::b:
				return (b_freq / pow(2, 8 - octal));
				break;
			default:
				return 0;
		};

	}

	PassiveBuzzer::Config getBuzzerConfig() {
		PassiveBuzzer::Config config;
		config.id = 0;
		return config;
	}
};


#endif /* INC_BUZZER_H_ */
