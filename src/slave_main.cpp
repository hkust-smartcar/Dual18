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
#include "flashWrapper.h"
#include "MenuV2.h"

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

	int cam_contrast = 0x40;
	int pre_contrast = 0x40;

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

	Joystick::State* joystick_state = nullptr;
	DualCarMenu menuV2(&lcd, &writer, Width, Height);
	DualCarMenu::SubMenu* current_page = &menuV2.home_page;

	menuV2.AddItem((char *) __TIME__, &(menuV2.home_page), false);
	menuV2.AddItem((char *) "start", &flash.imainboardID, &(menuV2.home_page), true);
	menuV2.AddItem((char *) "OpenMotor", menuV2.home_page.submenu_items[1].next_page, true);

	menuV2.AddItem((char *) "camera", &(menuV2.home_page), true);
	int right_corner_size = 0;
	menuV2.AddItem((char *) "Ctr", &cam_contrast, menuV2.home_page.submenu_items[2].next_page, true);
	menuV2.AddItem((char *) "Rcorner", &right_corner_size, menuV2.home_page.submenu_items[2].next_page, false);

	menuV2.AddItem((char *) "Flash", &(menuV2.home_page), true);
	menuV2.AddItem((char *) "mainboard", &flash.imainboardID, menuV2.home_page.submenu_items[3].next_page,
			true);
	menuV2.AddItem((char *) "MCUver", &flash.iMCUVer, menuV2.home_page.submenu_items[3].next_page, true);
	menuV2.AddItem((char *) "SaveConfig", [&flash]() {
		flash.saveConfigFromMenu();
	}, menuV2.home_page.submenu_items[3].next_page, false);
	menuV2.AddItem((char *) "SaveFlash", [&]() {
		flash.writeFlash();
	}, menuV2.home_page.submenu_items[3].next_page, false);
	menuV2.AddItem((char *) "LoadFlash", [&]() {
		flash.readFlash();
	}, menuV2.home_page.submenu_items[3].next_page, false);

	//joystick value

	Joystick js(myConfig::GetJoystickConfig(Joystick::Listener([&]
	(const uint8_t id, const Joystick::State state) {
		menuV2.SetJoystickState(state);
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
	uint32_t on9lastMain = 0;
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
	lcd.Clear();

	flash.link_int(0, &cam_contrast);
	flash.readFlash();

	while (1) {
		if (System::Time() != lastTime) {

			lastTime = System::Time();

			if (lastTime - on9lastMain >= cycle) {
				on9lastMain = lastTime;
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
				if (send_ms % 10 == 0) {
					send_ms = 0;
					led0.Switch();
				}

//				m_slave_bluetooth.send_edge(m_slave_vector);
				m_slave_bluetooth.send_slope(slave_slope);
				m_slave_bluetooth.send_corner(m_corner);
				m_slave_bluetooth.send_int_data(m_slave_vector.size(), Informations::edge_size);
				m_slave_bluetooth.send_int_data(mpuAccel[2], Informations::mpu);
				if (m_slave_vector.size() > 0) {
					m_slave_bluetooth.send_int_data(m_slave_vector[m_slave_vector.size() / 2].first,
							Informations::edge_xmid);
				} else {
					m_slave_bluetooth.send_int_data(0, Informations::edge_xmid);
				}

				////
				right_corner_size = m_corner.size();
				menuV2.SetCamBuffer(camBuffer);
				menuV2.SetEdge(m_slave_vector);
				menuV2.SetCorner(m_corner);
				current_page = menuV2.PrintSubMenu(current_page);
//				if(m_corner.size()>0){
//					int is_breakpoint;
//					is_breakpoint =1;
//				}
				m_corner.clear();
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
