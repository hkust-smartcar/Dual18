/*
 * slave_main.cpp
 *
 *  Created on: Jun 21, 2018
 *      Author: morristseng
 */

//#define slave

#ifdef slave

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
#include <libsc/k60/ov7725.h>
#include <libsc/k60/ov7725_configurator.h>
#include "corner.h"
#include "edge.h"
#include "bluetooth.h"
#include "DualCar_UART.h"
#include "libsc/passive_buzzer.h"
#include "libbase/k60/vectors.h"
#include "useful_functions.h"
#define pi 3.1415926

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

enum RoadType {
	straight, right_turn, left_turn
};

char *str = "";

//magnetic
int max(int a, int b) {
	return a > b ? a : b;
}
int min(int a, int b) {
	return a < b ? a : b;
}

//camera
#define Width 80
#define Height 60
bool camptr[Height][Width];

int stop = 0;
int mode = 0;

inline bool ret_cam_bit(int x, int y, const Byte* camBuffer) {
	return ((camBuffer[y * 10 + x / 8] >> (7 - (x % 8))) & 1); //return 1 if black
}

int main() {
	System::Init();
	Ov7725 camera(myConfig::getCameraConfig(Width, Height));
	camera.Start();

	Led led0(myConfig::GetLedConfig(0));
	Led led1(myConfig::GetLedConfig(1));
	Led led2(myConfig::GetLedConfig(2));
	Led led3(myConfig::GetLedConfig(3));

	led0.SetEnable(1);
	led1.SetEnable(1);
	led2.SetEnable(1);
	led3.SetEnable(1);

	PassiveBuzzer::Config config;
	PassiveBuzzer buzz(config);
	buzz.SetNote(523);

//	FutabaS3010 servo(myConfig::GetServoConfig());
//	AlternateMotor right_motor(myConfig::GetMotorConfig(1));
//	AlternateMotor left_motor(myConfig::GetMotorConfig(0));
	St7735r lcd(myConfig::GetLcdConfig());
	LcdTypewriter writer(myConfig::GetWriterConfig(&lcd));
	LcdConsole console(myConfig::GetConsoleConfig(&lcd));
//	DirEncoder REncoder(myConfig::GetEncoderConfig(0));
//	DirEncoder LEncoder(myConfig::GetEncoderConfig(1));
//	PID servoPIDStraight(2200, 0.03);
//	PID servoPIDCurve(5500, 1); //4825,0.5
//	PID servoPIDAlignCurve(-10, 0);
//	PID left_motorPID(0, 0, 0, &LEncoder, false);
//	PID right_motorPID(0, 0, 0, &REncoder, true);
	uint32_t lastTime = 0;
	uint8_t cycle = 10;
	uint8_t cycleTime = 0;

	//joystick value
	int line = 1;
	bool select = true;
	int select_time = 0;
	bool changed = false;

	Joystick js(myConfig::GetJoystickConfig(Joystick::Listener([&]
	(const uint8_t id, const Joystick::State state) {
		if(state == Joystick::State::kLeft) {
			changed = true;
			select = false;
			select_time = 0;
			line =1;
			if(mode == 0) {
				mode = 0;
			} else {
				mode--;
			}
		}
		else if(state == Joystick::State::kRight ) {
			changed = true;
			select = false;
			select_time = 0;
			line =1;
			if(mode == 3) {
				mode = 3;
			} else {
				mode++;
			}
		}
		else if (state == Joystick::State::kSelect) {
			if(mode ==1) {
			}
			if(mode ==2) {
			}
			if(mode==3) {
			}
		}
		else if (state == Joystick::State::kUp) {
			if(mode==0) {
			}
			else if(mode==1) {

			}
			else if(mode==2) {
			}
		}
		else if (state == Joystick::State::kDown) {
			if(mode==0) {
			}
			else if(mode==1) {

			}
			else if(mode==2) {
			}
		}
	})));

//	servo.SetDegree(1000);

	const bool is_slave = true;
	S_Bluetooth m_slave_bluetooth;
	vector<pair<int, int>> slave_min_xy;
	vector<pair<int, int>> slave_max_xy;
	uint32_t send_ms = 0;

	bool work = false;
	bool do_not_change_m = false;
	bool do_not_change_s = false;

	vector<pair<int, int>> m_slave_vector;
	vector<pair<int, int>> m_master_vector;
	vector<pair<int, int>> m_vector;
	int pre_x_coord = 40;

//	int servo_degree = 1000;

	while (1) {
		if (System::Time() != lastTime) {

			lastTime = System::Time();

			if (lastTime % cycle == 0) {
				const Byte* camBuffer = camera.LockBuffer();
				camera.UnlockBuffer();

// bluetooth send image

				double slave_slope;
				bool right_fail;
				vector<Corner> m_corner;
				right_fail = check_right_edge(20, 60, camBuffer,
						m_slave_vector);
				m_corner = check_corner(camBuffer, 20, 50, false);
				slave_slope = find_slope(m_slave_vector);
//				send_ms++;
//				if(send_ms%5==0) {
//					send_ms = 0;
				m_slave_bluetooth.send_edge(m_slave_vector);
				m_slave_bluetooth.send_info(right_fail);
				m_slave_bluetooth.send_corner(m_corner);
//				}

				if (mode == 0) {
					if (changed == 1) {
						lcd.Clear();
						changed = 0;
					}
					char c[10];
					for (int i = 0; i < 10; i++) {
						c[i] = ' ';
					}
					lcd.SetRegion(Lcd::Rect(0, 0, Width, Height));
					lcd.FillBits(0x0000, 0xFFFF, camBuffer, Width * Height);
					for (int i = 0; i < m_slave_vector.size(); i++) {
						lcd.SetRegion(
								Lcd::Rect(m_slave_vector[i].first,
										m_slave_vector[i].second, 2, 2));
						lcd.FillColor(Lcd::kBlue);
					}
					lcd.SetRegion(Lcd::Rect(0, 60, 88, 15));
					sprintf(c, "Slave");
					writer.WriteBuffer(c, 10);
					lcd.SetRegion(Lcd::Rect(0, 75, 88, 15));
					sprintf(c, "Sl:%.2f ", slave_slope);
					writer.WriteBuffer(c, 10);

					cycle = 10;
				}
				if (mode == 1) {
					if (changed == 1) {
						lcd.Clear();
						changed = 0;
					}

//					cycle = 50;
				}

				m_vector.clear();
				m_slave_vector.clear();
				m_master_vector.clear();

				cycleTime = System::Time() - lastTime;
			}
		}
	}
	return 0;
}

#endif

