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
#include <math.h>

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
#include "libsc/battery_meter.h"
#include "config.h"
#include "PID.h"
#include "bt.h"
#include "libsc/servo.h"
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
using libsc::Servo;

char *str = "";

#define MAG_MEAN_FILTER_NUM_OF_SMAPLES 8

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

int max(int a, int b) {
	return a > b ? a : b;
}

int min(int a, int b) {
	return a < b ? a : b;
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

	// magnetic variables
	Adc mag0(myConfig::GetAdcConfig(0));
	Adc mag1(myConfig::GetAdcConfig(1));
	Adc mag2(myConfig::GetAdcConfig(2));
	Adc mag3(myConfig::GetAdcConfig(3));

	std::vector<uint8_t> mag0_v;
	std::vector<uint8_t> mag1_v;
	std::vector<uint8_t> mag2_v;
	std::vector<uint8_t> mag3_v;

	uint16_t mag0_sum = 0;
	uint16_t mag1_sum = 0;
	uint16_t mag2_sum = 0;
	uint16_t mag3_sum = 0;

	mag0_v.clear();
	mag1_v.clear();
	mag2_v.clear();
	mag3_v.clear();

	for (uint8_t t = 0; t < MAG_MEAN_FILTER_NUM_OF_SMAPLES; t++) {
		mag0_v.emplace_back(0);
		mag1_v.emplace_back(0);
		mag2_v.emplace_back(0);
		mag3_v.emplace_back(0);
	}

	mag0.StartConvert();
	mag1.StartConvert();
	mag2.StartConvert();
	mag3.StartConvert();

	Servo servo(myConfig::GetServoConfig(0));
	AlternateMotor motorL(myConfig::GetMotorConfig(0));
	AlternateMotor motorR(myConfig::GetMotorConfig(1));
	St7735r lcd(myConfig::GetLcdConfig());
	LcdTypewriter writer(myConfig::GetWriterConfig(&lcd));
	LcdConsole console(myConfig::GetConsoleConfig(&lcd));
	lcd.SetRegion(Lcd::Rect(0, 0, 128, 160));
	lcd.FillColor(lcd.kWhite);
	BatteryMeter battery_meter(myConfig::GetBatteryMeterConfig());

	DirEncoder encoderL(myConfig::GetEncoderConfig(0));
	DirEncoder encoderR(myConfig::GetEncoderConfig(1));
	PID servoPID(1500, 0);
	PID motorLPID(0.3, 0.0, 0.0, &encoderL);
	PID motorRPID(0.6, 0.0, 0.0, &encoderR);
//	bt mBT(&servoPID, &motorLPID, &motorRPID);

	typedef enum {
		normal = 0, nearLoop, turning, inside, exit1, exit2
	}carState;

	carState state = normal;
	bool start = false;
	uint32_t lastTime = 0;
	uint8_t left_mag, right_mag, mid_left, mid_right;
//	uint32_t left_sum = 0, right_sum = 0, cycleCount = 0;
	float angle = 0;
	float left_x, right_x;
	const float left_a = 167, left_b = -0.165, right_a = 257, right_b = -0.218;
	const float h = 6.2;
	float magRatio, xRatio;
	bool left = 0;
	uint8_t count = 0;
	bool waitTrigger = 1, detectLoop = 0;
//	uint16_t graphCount = 0;

	Joystick js(
			myConfig::GetJoystickConfig(
					Joystick::Listener(
							[&](const uint8_t id, const Joystick::State state) {
								if (state == Joystick::State::kSelect) {
									start = true;
									motorLPID.setDesiredVelocity(40);
									motorRPID.setDesiredVelocity(40);
								}
								else {
									start = false;
									motorL.SetPower(0);
									motorR.SetPower(0);
									motorLPID.setDesiredVelocity(0);
									motorRPID.setDesiredVelocity(0);
								}
								led2.Switch();
							})));

	servo.SetDegree(900);
	uint16_t servo_mid = 900;

	float battery = battery_meter.GetVoltage();
	lcd.SetRegion(Lcd::Rect(0, 0, 128, 160));
	if (battery > 8) {
		lcd.FillColor(0xF100);
	} else if (battery > 7.7) {
		lcd.FillColor(0x0EE0);
	} else if (battery > 7.5) {
		lcd.FillColor(0x001F);
	} else {
		lcd.FillColor(0xFFFF);
	}
	char c[10];
	lcd.SetRegion(Lcd::Rect(0, 0, 128, 15));
	sprintf(c, "0  ");
	writer.WriteBuffer(c, 10);
	lcd.SetRegion(Lcd::Rect(0, 50, 128, 15));
	sprintf(c, "50 ");
	writer.WriteBuffer(c, 10);
	lcd.SetRegion(Lcd::Rect(0, 100, 128, 15));
	sprintf(c, "100");
	writer.WriteBuffer(c, 10);

	while (1) {
		if (System::Time() != lastTime) {
			lastTime = System::Time();

			// magnetic variables
			mag0_sum -= mag0_v[0];
			mag0_v.erase(mag0_v.begin());
			mag0_v.emplace_back(mag0.GetResult());
			mag0_sum += mag0_v[MAG_MEAN_FILTER_NUM_OF_SMAPLES-1];

			mag1_sum -= mag1_v[0];
			mag1_v.erase(mag1_v.begin());
			mag1_v.emplace_back(mag1.GetResult());
			mag1_sum += mag1_v[MAG_MEAN_FILTER_NUM_OF_SMAPLES-1];

			mag2_sum -= mag2_v[0];
			mag2_v.erase(mag2_v.begin());
			mag2_v.emplace_back(mag2.GetResult());
			mag2_sum += mag2_v[MAG_MEAN_FILTER_NUM_OF_SMAPLES-1];

			mag3_sum -= mag3_v[0];
			mag3_v.erase(mag3_v.begin());
			mag3_v.emplace_back(mag3.GetResult());
			mag3_sum += mag3_v[MAG_MEAN_FILTER_NUM_OF_SMAPLES-1];

//    		if(lastTime % 100 == 0){
//				mBT.sendVelocity();
//    		}

			if (lastTime % 3 == 0) {
				left_mag = mag0_sum / MAG_MEAN_FILTER_NUM_OF_SMAPLES;
				right_mag = mag1_sum / MAG_MEAN_FILTER_NUM_OF_SMAPLES;
				mid_left = mag2_sum / MAG_MEAN_FILTER_NUM_OF_SMAPLES;
				mid_right = mag3_sum / MAG_MEAN_FILTER_NUM_OF_SMAPLES;
//				cycleCount++;
//				left_sum += left_mag;
//				right_sum += right_mag;
//				left_x = log(left_mag / left_a) / left_b;
//				right_x = log(right_mag / right_a) / right_b;
//				xRatio = (float) left_x / (right_x + left_x);
				xRatio = (float)left_mag/(right_mag+left_mag);

				angle = servoPID.getPID(0.5, xRatio);
				angle += servo_mid;
				angle = min(max(angle, 600), 1500);
				servo.SetDegree(angle);

//				if (count==0 && ((mid_left>120 && mid_right>40 && left_mag>right_mag) || (mid_right>120 && mid_left>40 && right_mag>left_mag)) && mid_left + mid_right > 150) {
//					detectLoop = 1;
//					if (mid_left > 100 && mid_right > 40
//							&& left_mag > right_mag) {
//						left = 1;
//					} else {
//						left = 0;
//					}
//					state = nearLoop;
//				}
//				if (detectLoop && state!=turning && waitTrigger && left_x+right_x<=17 && (left_mag>90 || right_mag>90)) {
//					count++;
//					waitTrigger = 0;
//					if (count == 2) {
//						state = turning;
//					} else if (count > 2 && state == inside) {
//						state = exit1;
//						lcd.SetRegion(Lcd::Rect(0, 0, 128, 10));
//						lcd.FillColor(0x0EE0);
//					}
//				}
//				if (!waitTrigger && left_mag<=75 && right_mag<=75 && left_mag+right_mag<=135) {
//					waitTrigger = 1;
//				}
//				if (!waitTrigger && state==exit1 && left_mag<=90 && right_mag<=90 && 0.3<xRatio && xRatio<0.7 && left_x+right_x>10) {
//					state = exit2;
//					lcd.SetRegion(Lcd::Rect(0, 0, 128, 10));
//					lcd.FillColor(0x001F);
//				}
//				if (state==exit2 && left_x+right_x>20) {
//					detectLoop = 0;
//					state = normal;
//					count = 0;
//					lcd.SetRegion(Lcd::Rect(0, 0, 128, 10));
//					lcd.FillColor(0xFFFF);
//				}
//
//				angle = servoPID.getPID(0.5, xRatio);
//				angle += servo_mid;
//				angle = min(max(angle, 0), 1800);
//				if (state == turning) {
//					if (left_x+right_x>20) {
//						state = inside;
//					}
//					if (left) {
//						servo.SetDegree(
//								angle < servo_mid + 350 ?
//										servo_mid + 700 : angle);
//					} else {
//						servo.SetDegree(
//								angle > servo_mid - 350 ?
//										servo_mid - 700 : angle);
//					}
//				}
//				if (state == normal || state == inside) {
//					servo.SetDegree(angle);
//				} else if (state == nearLoop) {
//					if (left) {
//						servo.SetDegree(
//								angle > servo_mid + 150 ?
//										servo_mid + 150 : angle);
//					} else {
//						servo.SetDegree(
//								angle < servo_mid - 150 ?
//										servo_mid - 150 : angle);
//					}
//				} else if (state == exit1) {
//					if (!left) {
//						servo.SetDegree(
//								angle > servo_mid + 450 ?
//										servo_mid + 450 : angle);
//					} else {
//						servo.SetDegree(
//								angle < servo_mid - 450 ?
//										servo_mid - 450 : angle);
//					}
//				} else if (state == exit2) {
//					if (left) {
//						servo.SetDegree(
//								angle > servo_mid + 100 ?
//										servo_mid + 100 : angle);
//					} else {
//						servo.SetDegree(
//								angle < servo_mid - 100 ?
//										servo_mid - 100 : angle);
//					}
//				}

				if (start){
					motorL.SetPower(min(350, max(motorLPID.getPID(0), 90)));
					motorR.SetPower(min(350, max(motorRPID.getPID(0), 90)));
				}
			}

			char c[10];
			if (lastTime % 100 == 0) {
				if (start) {
				}
				else {
					led3.Switch();
					lcd.SetRegion(Lcd::Rect(0, 0, 128, 15));
					sprintf(c, "R: %d", left_mag);
					writer.WriteBuffer(c, 10);
					lcd.SetRegion(Lcd::Rect(0, 15, 128, 15));
					sprintf(c, "R: %d", right_mag);
					writer.WriteBuffer(c, 10);
					lcd.SetRegion(Lcd::Rect(0, 30, 128, 15));
					sprintf(c, "M: %d", mid_left);
					writer.WriteBuffer(c, 10);
					lcd.SetRegion(Lcd::Rect(0, 45, 128, 15));
					sprintf(c, "M: %d", mid_right);
					writer.WriteBuffer(c, 10);
					lcd.SetRegion(Lcd::Rect(0, 60, 128, 15));
					sprintf(c, "X: %f", left_x);
					writer.WriteBuffer(c, 10);
					lcd.SetRegion(Lcd::Rect(0, 75, 128, 15));
					sprintf(c, "X: %f", right_x);
					writer.WriteBuffer(c, 10);
					lcd.SetRegion(Lcd::Rect(0, 90, 128, 15));
					sprintf(c, "A: %d", servo.GetDegree());
					writer.WriteBuffer(c, 10);
//					lcd.SetRegion(Lcd::Rect(0, 105, 128, 15));
//					if (state == normal) {
//						lcd.FillColor(0xFF00);
//					} else if (state == exit1) {
//						lcd.FillColor(0xF100);
//					} else if (state == exit2) {
//						lcd.FillColor(0x001F);
//					} else if (state == nearLoop) {
//						if (!left) {
//							lcd.SetRegion(Lcd::Rect(64, 105, 64, 15));
//						} else {
//							lcd.SetRegion(Lcd::Rect(0, 105, 64, 15));
//						}
//						lcd.FillColor(0x0FF0);
//					} else if (state == turning) {
//						lcd.FillColor(0x00FF);
//					} else if (state == inside) {
//						lcd.FillColor(0x0000);
//					}
					lcd.SetRegion(Lcd::Rect(0, 120, 128, 15));
					sprintf(c, "X: %f", (float) xRatio);
					writer.WriteBuffer(c, 10);

//					lcd.SetRegion(Lcd::Rect(graphCount+28,0,1,160));
//					lcd.FillColor(0x0000);
//					lcd.SetRegion(Lcd::Rect(graphCount+28,left_mag,1,1));
//					lcd.FillColor(0xFFFF);
//					lcd.SetRegion(Lcd::Rect(graphCount+28,right_mag,1,1));
//					lcd.FillColor(0xF100);
//					lcd.SetRegion(Lcd::Rect(graphCount+28,mid_right,1,1));
//					lcd.FillColor(0x0EE0);
//					graphCount = (graphCount+1) % 100;
				}
			}
		}
	}
	return 0;
}

