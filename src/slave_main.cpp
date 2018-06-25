/*
 * slave_main.cpp
 *
 *  Created on: Jun 21, 2018
 *      Author: morristseng
 */

#define slave

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
#include "menu.h"
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

	St7735r lcd(myConfig::GetLcdConfig());
	LcdTypewriter writer(myConfig::GetWriterConfig(&lcd));
	LcdConsole console(myConfig::GetConsoleConfig(&lcd));
	uint32_t lastTime = 0;
	uint8_t cycle = 10;
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
		else if(state == Joystick::State::kLeft){
			menu.change_mode(false);
		}
		else if(state == Joystick::State::kSelect){
			menu.select_pressed();
		}
		else if(state == Joystick::State::kUp){
			menu.change_line(true);

		}
		else if(state == Joystick::State::kDown){
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

//	int servo_degree = 1000;



	while (1) {
		if (System::Time() != lastTime) {

			lastTime = System::Time();

			if (lastTime % cycle == 0) {
				const Byte* camBuffer = camera.LockBuffer();
				camera.UnlockBuffer();

// bluetooth send image

				float slave_slope;
				bool right_fail;
				vector<Corner> m_corner;
				right_fail = check_right_edge(20, 60, camBuffer,
						m_slave_vector);
				m_corner = check_corner(camBuffer, 20, 50, false);
				slave_slope = find_slope(m_slave_vector);
				send_ms++;
				if(send_ms%2==0) {
					send_ms = 0;
				m_slave_bluetooth.send_edge(m_slave_vector);
				m_slave_bluetooth.send_info(right_fail);
				m_slave_bluetooth.send_corner(m_corner);
				}


				if(menu.get_mode()!=2){
					Items item0 ("Slave");
					Items item1("Sl", slave_slope);
					Items item2("Select", menu.get_selected());
					Items item3("line", menu.get_line());
					Items item4("corner", m_corner.size());

					mode0.add_items(&item0);
					mode0.add_items(&item1);
					mode0.add_items(&item2);
					mode0.add_items(&item3);
					mode0.add_items(&item4);

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
						lcd.FillColor(Lcd::kBlue);
					}

					for (int i = 0; i < m_corner.size(); i++) {
						lcd.SetRegion(
								Lcd::Rect(m_corner[i].get_xcoord(),
										m_corner[i].get_ycoord(), 2, 2));
						lcd.FillColor(Lcd::kRed);
					}

					for(int i=0; i<menu.m_menu[menu.get_mode()]->get_max_line(); i++){
						lcd.SetRegion(Lcd::Rect(0, 60+15*i, 88, 15));
						writer.WriteBuffer(menu.m_menu[0]->m_items[i]->get_message(),15);
					}
				}
				else if (menu.get_mode() == 1) {
					if (menu.change_screen()) {
						lcd.Clear();
					}
					for(int i=0; i<menu.m_menu[menu.get_mode()]->get_max_line(); i++){
						lcd.SetRegion(Lcd::Rect(0, 15*i, 88, 15));
						writer.WriteBuffer(menu.m_menu[menu.get_mode()]->m_items[i]->get_message(),15);
					}
				}
				else{
					if (menu.change_screen()) {
						lcd.Clear();
					}
				}
				menu.clear();
				mode0.clear();
				mode1.clear();
				m_slave_vector.clear();

				cycleTime = System::Time() - lastTime;
			}
		}
	}
	return 0;
}

#endif

