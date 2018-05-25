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
#include "libsc/servo.h"
#include "PID.h"


#include "../inc/algorithm/testing_car1_board1.h"
#include "config.h"




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

using namespace libsc;
using namespace libbase::k60;

using libsc::BatteryMeter;
using libsc::Lcd;
using libsc::LcdConsole;
using libsc::St7735r;
using libsc::System;
using libsc::Joystick;

enum struct Algorithm {
  kOptimal,
  kTest
};


int main() {
	  System::Init();
	  BatteryMeter::Config ConfigBM;
	  ConfigBM.voltage_ratio = 0.4;
	  BatteryMeter bm(ConfigBM);
	  // Battery Check
	  {
	    St7735r::Config lcd_config;
	    lcd_config.orientation = 0;
	    St7735r lcd(lcd_config);
	    lcd.Clear();

	    LcdConsole::Config console_config;
	    console_config.lcd = &lcd;
	    console_config.region=libsc::Lcd::Rect(0,0,lcd.GetW(),lcd.GetH());
	    LcdConsole console(console_config);

	    float voltage;
	    do {
	      voltage = bm.GetVoltage();
	      console.SetTextColor(voltage <= 7.4 ? Lcd::kRed : Lcd::kGreen);
	      char temp[32];
	      sprintf(temp, " Voltage: %.2fV", voltage);
	      console.WriteString(temp);
	      System::DelayMs(1000);
	    } while (voltage <= 7.4);

	  }

	  // modify next line to switch between algorithms
	  constexpr Algorithm a = Algorithm::kTest;
//
//	  bool reset = false;
//	  {
//	    Joystick::Config joystick_config;
//	    joystick_config.id = 0;
//	    joystick_config.is_active_low = true;
//	    Joystick joystick(joystick_config);
//
//	    reset = joystick.GetState() == Joystick::State::kSelect;
//	  }

	 // uint16_t car = debug(reset); TODO: screen for select/reset
	  uint16_t car=1;
	  uint16_t board=1;

	  switch (a) {
	    case Algorithm::kOptimal:
	    {
	      switch (car) {
//	        case 1:
//	          algorithm::optimal::car1::main_car1(debug_flag::lcd_debug);
//	          break;
//	        case 2:
//	          algorithm::optimal::car2::main_car2(debug_flag::lcd_debug);
//	          break;
	        default:
	          // not handled
	          break;
	      }
	      break;
	    }
	    case Algorithm::kTest:
	    {
		  switch (car) {
			case 1:
			{
				switch(board)
				{
				case 1:
					algorithm::testing::car1::board1::main_car1_board1();
					break;
				case 2:
					break;
				}
				break;
			}
			case 2:
			{
				switch(board)
				{
				case 1:
					break;
				case 2:
					break;
				}
				break;
			}
		  }
		  break;
	    }
	    default:
	      // all cases covered
	      break;
	  }

	  while (true);
	  return 0;
}







