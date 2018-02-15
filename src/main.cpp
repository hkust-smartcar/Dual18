/*
 * main.cpp
 *
 * Author: 
 * Copyright (c) 2014-2015 HKUST SmartCar Team
 * Refer to LICENSE for details
 */

#include <cmath>
#include <vector>
#include <cassert>
#include <cstring>
#include <string>
#include <libbase/k60/mcg.h>
#include <libsc/system.h>

#include <libsc/led.h>
#include <libsc/k60/jy_mcu_bt_106.h>
#include "libsc/led.h"
#include "libsc/joystick.h"
#include "libsc/st7735r.h"
#include "libsc/battery_meter.h"
#include "libsc/k60/jy_mcu_bt_106.h"
#include "libbase/k60/pit.h"
#include "libsc/lcd_typewriter.h"
#include "libbase/k60/adc.h"
#include "config.h"
#include "PID.h"
#include "bt.h"
namespace libbase {
namespace k60 {
Mcg::Config Mcg::GetMcgConfig() {
	Mcg::Config config;
	config.external_oscillator_khz = 50000;
	config.core_clock_khz = 150000;
	return config;
}
}
}

using libsc::System;
using namespace libsc;
using namespace libbase::k60;
using libsc::k60::JyMcuBt106;

const float left_k = 767.2497;
const float right_k = 854.7614;
const float h = 6.2;

float inv_sqrt(float x) {
	float xhalf = 0.5f * x;
	int i = *(int*) &x;
	i = 0x5f3759df - (i >> 1);
	x = *(float*) &i;
	x = x * (1.5f - xhalf * x * x);
	x = x * (1.5f - xhalf * x * x);
	return x;
}

float _sqrt(float x) {
	return x * inv_sqrt(x);
}

void straightLineFollowing(const uint8_t &left_mag, const uint8_t &right_mag,
		PID &servoPID, float &angle) {

	float xRatio;
	float left_x, right_x;

	if (left_k * h / left_mag < h * h) {
		left_x = 0;
	} else {
		left_x = _sqrt(left_k * h / left_mag - h * h);
	}

	if (right_k * h / right_mag < h * h) {
		right_x = 0;
	} else {
		right_x = _sqrt(right_k * h / right_mag - h * h);
	}

	xRatio = ((float) left_x) / (right_x + left_x);
	angle = servoPID.getPID(0.5, xRatio);
	angle += 900;
}

// history
std::vector<uint8_t> LCD_mag_history_left;
std::vector<uint8_t> LCD_mag_history_right;
std::vector<uint8_t> LCD_mag_history_mid;
uint8_t LCD_mag_history_cursor = 0;
inline void LCD_mag_history(St7735r &lcd, const uint8_t &left_mag,
		const uint8_t &right_mag, const uint8_t &mid_mag) {

	const uint8_t yOffset = 160;

	// update cursor
	LCD_mag_history_cursor += 1;
	LCD_mag_history_cursor =
			LCD_mag_history_cursor == 128 ? 0 : LCD_mag_history_cursor;

	// remove last point first
	lcd.SetRegion(
			Lcd::Rect(LCD_mag_history_cursor,
					-LCD_mag_history_left[LCD_mag_history_cursor] + yOffset, 1,
					1));
	lcd.FillColor(Lcd::kWhite);

	lcd.SetRegion(
			Lcd::Rect(LCD_mag_history_cursor,
					-LCD_mag_history_right[LCD_mag_history_cursor] + yOffset, 1,
					1));
	lcd.FillColor(Lcd::kWhite);

	lcd.SetRegion(
			Lcd::Rect(LCD_mag_history_cursor,
					-LCD_mag_history_mid[LCD_mag_history_cursor] + yOffset, 1,
					1));
	lcd.FillColor(Lcd::kWhite);

	// update the array
	LCD_mag_history_left[LCD_mag_history_cursor] = left_mag;
	LCD_mag_history_right[LCD_mag_history_cursor] = right_mag;
	LCD_mag_history_mid[LCD_mag_history_cursor] = mid_mag;

	// write new Pt
	lcd.SetRegion(
			Lcd::Rect(LCD_mag_history_cursor,
					-LCD_mag_history_left[LCD_mag_history_cursor] + yOffset, 1,
					1));
	lcd.FillColor(Lcd::kPurple);

	lcd.SetRegion(
			Lcd::Rect(LCD_mag_history_cursor,
					-LCD_mag_history_right[LCD_mag_history_cursor] + yOffset, 1,
					1));
	lcd.FillColor(Lcd::kBlack);

	lcd.SetRegion(
			Lcd::Rect(LCD_mag_history_cursor,
					-LCD_mag_history_mid[LCD_mag_history_cursor] + yOffset, 1,
					1));
	lcd.FillColor(Lcd::kRed);
}

// this function prints the bar
inline void LCD_bar(St7735r &lcd, const uint16_t color) {
	lcd.SetRegion(Lcd::Rect(0, 15, 128, 15));
	lcd.FillColor(color);
}

// this fucntion prints the value of xRatio
float past_xRatio = 0;
inline void LCD_scale(St7735r &lcd, const uint8_t &left_mag,
		const uint8_t &right_mag) {

	// calculate the value

	float xRatio;
	float left_x, right_x;

	if (left_k * h / left_mag < h * h) {
		left_x = 0;
	} else {
		left_x = _sqrt(left_k * h / left_mag - h * h);
	}

	if (right_k * h / right_mag < h * h) {
		right_x = 0;
	} else {
		right_x = _sqrt(right_k * h / right_mag - h * h);
	}

	xRatio = ((float) left_x) / (right_x + left_x);

	// print the scale

	if (xRatio != past_xRatio) {
		// remvoe the new strip
		lcd.SetRegion(Lcd::Rect(past_xRatio * 128 - 1, 0, 3, 15));
		lcd.FillColor(Lcd::kWhite);

		// print the new strip
		lcd.SetRegion(Lcd::Rect(xRatio * 128 - 1, 0, 3, 15));
		lcd.FillColor(Lcd::kPurple);

		past_xRatio = xRatio;
	}
}

int main() {
	System::Init();

	Led led0(myConfig::GetLedConfig(0));
	Led led1(myConfig::GetLedConfig(1));
	Led led2(myConfig::GetLedConfig(2));
	Led led3(myConfig::GetLedConfig(3));

	led0.SetEnable(1);
	led1.SetEnable(1);
	led2.SetEnable(1);
	led3.SetEnable(1);

	Adc mag0(myConfig::GetAdcConfig(0));
	Adc mag1(myConfig::GetAdcConfig(1));
	Adc mag2(myConfig::GetAdcConfig(2));
	Adc mag3(myConfig::GetAdcConfig(3));

	mag0.StartConvert();
	mag1.StartConvert();
	mag2.StartConvert();
	mag3.StartConvert();

	FutabaS3010 servo(myConfig::GetServoConfig());
	AlternateMotor motor(myConfig::GetMotorConfig());

	St7735r lcd(myConfig::GetLcdConfig());
	LcdTypewriter writer(myConfig::GetWriterConfig(&lcd));
	LcdConsole console(myConfig::GetConsoleConfig(&lcd));

	DirEncoder dirEncoder(myConfig::GetEncoderConfig());
	PID servoPID(2800, 40000);
	PID motorLPID(0.3, 0.0, 0.0, &dirEncoder);
	PID motorRPID(0.6, 0.0, 0.0, &dirEncoder);

	uint32_t lastTime = 0;
	uint16_t motorPW = 0;

	float angle = 0;

	typedef enum {
		normal = 0, nearLoop, straight, turning, transit
	} carState;

	carState currentState = normal;

	Joystick js(
			myConfig::GetJoystickConfig(
					Joystick::Listener(
							[&](const uint8_t id, const Joystick::State state) {
								if (state == Joystick::State::kSelect) {
									motorPW = motorPW == 0 ? 110 : 0;
								}

								if (id == 0) {
									// with this line, no more warning la xxxxxd ^^
								}
							})));

	servo.SetDegree(900);

	// init lcd
	lcd.SetRegion(Lcd::Rect(0, 0, 128, 160));
	lcd.FillColor(Lcd::kWhite);

	for (uint8_t i = 0; i < 128; i++) {
		LCD_mag_history_left.emplace_back(0);
		LCD_mag_history_right.emplace_back(0);
		LCD_mag_history_mid.emplace_back(0);
	}

	while (true) {
		if (System::Time() != lastTime) {
			lastTime = System::Time();

			if (lastTime % 1000 == 0) {
				led1.Switch();
			}

			if (lastTime % 100 == 0) {
				// get sensor values
				uint8_t left_mag, right_mag;

				left_mag = mag0.GetResult();
				right_mag = mag1.GetResult();
				mid_mag = mag2.GetResult();

				// graphics
				LCD_scale(lcd, left_mag, right_mag);
			}

			if (lastTime % 10 == 0) {
				// get sensor values
				uint8_t left_mag, right_mag, mid_mag;

				left_mag = mag0.GetResult();
				right_mag = mag1.GetResult();
				mid_mag = mag2.GetResult();

				LCD_mag_history(lcd, left_mag, right_mag, mid_mag);

				// state changer
				// state change here
				switch ((int) currentState) {
				case carState::normal:

					if (left_mag > 80 && mid_mag > 80) {
						currentState = nearLoop;
					}

					LCD_bar(lcd, Lcd::kGreen);

					straightLineFollowing(left_mag, right_mag, servoPID, angle);

					break;

				case carState::nearLoop:
					if (left_mag < 50) {
						currentState = straight;
					}

					LCD_bar(lcd, Lcd::kBlue);

					angle = 900;
					break;

				case carState::straight:
					if (left_mag > 80) {
						currentState = transit;
					}

					LCD_bar(lcd, Lcd::kYellow);

					straightLineFollowing(left_mag, right_mag, servoPID, angle);

					break;

				case carState::transit:
					if (left_mag < 40 && right_mag < 40) {
						currentState = normal;
					}

					LCD_bar(lcd, Lcd::kPurple);

					angle = 1700;
					break;

				case carState::turning:
					break;

				default:
					break;
				}

				// servo limit
				if (angle > 1800) {
					angle = 1800;
				} else if (angle < 0) {
					angle = 0;
				}
				servo.SetDegree(angle);

				motor.SetPower(motorPW);

				if (lastTime % 100 == 0) {
					led3.Switch();
				}
			}
		}
	}

	return 0;
}
