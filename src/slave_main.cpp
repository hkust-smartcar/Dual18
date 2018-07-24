/*
 * slave_main.cpp
 *
 *  Created on: Jun 21, 2018
 *      Author: morristseng
 */

//#define slave

#ifdef slave

//#define temp_cam_fix
//#define use_mpu

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
#include "menu.h"
#include "libsc/mpu6050.h"
#include "FlashWrapper.h"

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

	FlashWrapper flash;
//	flash.setAsSlave();
//	flash.setMCUVer(2);

	St7735r lcd(myConfig::GetLcdConfig());
	lcd.SetRegion(Lcd::Rect(0, 0, 128, 160));
	lcd.FillColor(0x00FF);
	LcdTypewriter writer(myConfig::GetWriterConfig(&lcd));
	LcdConsole console(myConfig::GetConsoleConfig(&lcd));

	{
		lcd.SetRegion(Lcd::Rect(0, 0, 80, 15));
		char t[15];
		sprintf(t, "init camera");
		writer.WriteBuffer(t, 15);
	}

	Ov7725 camera(myConfig::getCameraConfig(Width, Height, flash.getCameraConfig()));
	camera.Start();

	{
		lcd.SetRegion(Lcd::Rect(0, 15, 80, 15));
		char t[15];
		sprintf(t, "camera init done");
		writer.WriteBuffer(t, 15);
	}

	Led led0(myConfig::GetLedConfig(flash.getLEDConfig(0)));
	Led led1(myConfig::GetLedConfig(flash.getLEDConfig(1)));

//	Led led0(myConfig::GetLedConfig(2));
//	Led led1(myConfig::GetLedConfig(0));

//	Led led2(myConfig::GetLedConfig(2));
//	Led led3(myConfig::GetLedConfig(3));

	Edge right_edge(true, 25, 60);

	led0.SetEnable(1);
	led1.SetEnable(1);
//	led2.SetEnable(1);
//	led3.SetEnable(1);

	PassiveBuzzer::Config config;
	PassiveBuzzer buzz(config);
	buzz.SetNote(523);

	uint32_t lastTime = 0;
	uint8_t cycle = 12;
	uint8_t cycleTime = 0;

	//joystick value
	DualCar_Menu menu;
	Mode mode0(0);
	Mode mode1(1);
	Mode ClearMode(2);

	Joystick js(myConfig::GetJoystickConfig(Joystick::Listener([&]
	(const uint8_t id, const Joystick::State state) {
		if(state == Joystick::State::kRight) {
			menu.change_mode(true);
		}
		else if(state == Joystick::State::kLeft) {
			menu.change_mode(false);
		}
		else if(state == Joystick::State::kSelect) {
			menu.select_pressed();
		}
		else if(state == Joystick::State::kUp) {
			menu.change_line(true);

		}
		else if(state == Joystick::State::kDown) {
			menu.change_line(false);
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
	int pre_x_coord = 40;
	led0.SetEnable(0);
//	int servo_degree = 1000;

	// init mpu

	{
		lcd.SetRegion(Lcd::Rect(0, 30, 80, 15));
		char t[15];
		sprintf(t, "init mpu");
		writer.WriteBuffer(t, 15);
	}

#ifdef use_mpu
	Mpu6050::Config MpuConfig;
	MpuConfig.accel_range = Mpu6050::Config::Range::kExtreme;
	MpuConfig.gyro_range = Mpu6050::Config::Range::kExtreme;
	MpuConfig.cal_drift = true;
	Mpu6050 mpu(MpuConfig);
#endif

	bool led_toggle = true;

	{
		lcd.SetRegion(Lcd::Rect(0, 45, 80, 15));
		char t[15];
		sprintf(t, "mpu init done");
		writer.WriteBuffer(t, 15);
	}

	{
		lcd.SetRegion(Lcd::Rect(0, 60, 80, 15));
		char t[15];
		sprintf(t, "delay 200 ms...");
		writer.WriteBuffer(t, 15);
	}

	System::DelayMs(200);

	{
		lcd.SetRegion(Lcd::Rect(0, 75, 80, 15));
		char t[15];
		sprintf(t, "delay ended");
		writer.WriteBuffer(t, 15);
	}

	while (1) {
		if (System::Time() != lastTime) {

			lastTime = System::Time();

			if (lastTime % cycle == 0) {
#ifdef temp_cam_fix
				const Byte* camBuffer_Original = camera.LockBuffer();
				camera.UnlockBuffer();

				Byte * camBuffer = new Byte[Width * Height / 8];
				for (uint16_t i = 0; i < Width * Height / 8; i++) {
					Byte t = *(camBuffer_Original + i);
					*(camBuffer + i) = t << 3 | t >> 5;
				}
#else
				const Byte* camBuffer = camera.LockBuffer();
				camera.UnlockBuffer();
#endif

// bluetooth send image

				//mpu
#ifdef use_mpu
				mpu.Update(1);
				std::array<int32_t, 3> mpuAccel = mpu.GetAccel();
				mpuAccel[2] -= 2630;
#else
				std::array<int32_t, 3> mpuAccel;
				mpuAccel[2] = 0;
#endif
				//

				float slave_slope;
				bool right_fail;
				m_slave_vector = right_edge.check_edge(camBuffer);
				vector<Corner> m_corner;
				m_corner = check_cornerv2(camBuffer, 25, 60, m_slave_vector);
				slave_slope = find_slope(m_slave_vector);
				send_ms++;
				if(send_ms%10==0) {
					send_ms = 0;
					led0.Switch();
				}


//				m_slave_bluetooth.send_edge(m_slave_vector);
				m_slave_bluetooth.send_slope(slave_slope);
				m_slave_bluetooth.send_corner(m_corner);
				m_slave_bluetooth.send_int_data(m_slave_vector.size(),
						Informations::edge_size);
				m_slave_bluetooth.send_int_data(mpuAccel[2], Informations::mpu);
				if (m_slave_vector.size() > 0) {
					m_slave_bluetooth.send_int_data(
							m_slave_vector[m_slave_vector.size() / 2].first,
							Informations::edge_xmid);
				} else {
					m_slave_bluetooth.send_int_data(0, Informations::edge_xmid);
				}

//				}

				if (menu.get_mode() != 2) {
					Items item0("Slave");
					Items item1("Sl", slave_slope);
//					Items item2("Select", menu.get_selected());
//					Items item3("line", menu.get_line());
					Items item2("corner", m_corner.size());
					Items item3("S_es", m_slave_vector.size());
					Items item4("xmid",
							m_slave_vector[m_slave_vector.size() / 2].first);
					Items item5("xmid", 0);
					Items item6("mpu", mpuAccel[2]);

					mode0.add_items(&item0);
					mode0.add_items(&item1);
					mode0.add_items(&item2);
					mode0.add_items(&item3);
					mode0.add_items(&item6);
					if (m_slave_vector.size() > 0)
						mode0.add_items(&item4);
					else
						mode0.add_items(&item5);

				}

				menu.add_mode(&mode0);
				menu.add_mode(&mode1);
				menu.add_mode(&ClearMode);

				if (menu.get_mode() == 0) {

					if (menu.change_screen()) {
						lcd.Clear();
					}
					lcd.SetRegion(Lcd::Rect(0, 0, Width, Height));
					lcd.FillBits(0x0000, 0xFFFF, camBuffer, Width * Height);
					for (int i = 0; i < m_slave_vector.size(); i++) {
						lcd.SetRegion(
								Lcd::Rect(m_slave_vector[i].first,
										m_slave_vector[i].second, 2, 2));
						lcd.FillColor(Lcd::kYellow);
					}

					for (int i = 0; i < m_corner.size(); i++) {
						lcd.SetRegion(
								Lcd::Rect(m_corner[i].get_xcoord(),
										m_corner[i].get_ycoord(), 2, 2));
						lcd.FillColor(Lcd::kBlue);
					}

					if(m_slave_vector.size()>0){
						lcd.SetRegion(Lcd::Rect(m_slave_vector[0].first,m_slave_vector[0].second, 2, 2));
						lcd.FillColor(Lcd::kRed);
						lcd.SetRegion(Lcd::Rect(m_slave_vector[m_slave_vector.size()-1].first,m_slave_vector[m_slave_vector.size()-1].second, 2, 2));
						lcd.FillColor(Lcd::kGreen);
					}
					for (int i = 0;
							i < menu.m_menu[menu.get_mode()]->get_max_line();
							i++) {
						lcd.SetRegion(Lcd::Rect(0, 60 + 15 * i, 88, 15));
						writer.WriteBuffer(
								menu.m_menu[0]->m_items[i]->get_message(), 15);
					}
				} else if (menu.get_mode() == 1) {
					if (menu.change_screen()) {
						lcd.Clear();
					}
					for (int i = 0;
							i < menu.m_menu[menu.get_mode()]->get_max_line();
							i++) {
						lcd.SetRegion(Lcd::Rect(0, 15 * i, 88, 15));
						writer.WriteBuffer(
								menu.m_menu[menu.get_mode()]->m_items[i]->get_message(),
								15);
					}
				} else {
					if (menu.change_screen()) {
						lcd.Clear();
					}
				}
//				if(m_corner.size()>0){
//					int is_breakpoint;
//					is_breakpoint =1;
//				}
				m_corner.clear();
				menu.clear();
				mode0.clear();
				mode1.clear();
				m_slave_vector.clear();

#ifdef temp_cam_fix
				delete[] camBuffer;
#endif

				cycleTime = System::Time() - lastTime;
			}
		}
	}
	return 0;
}

#endif
