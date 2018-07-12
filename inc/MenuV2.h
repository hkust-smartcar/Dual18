/*
 * MenuV2.h
 *
 *  Created on: Jul 9, 2018
 *      Author: morristseng
 */

#ifndef INC_MENUV2_H_
#define INC_MENUV2_H_

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
#include "config.h"
#include <libbase/k60/flash.h>
#include <cstring>
using namespace std;

class DualCarMenu{
public:
	enum MessageType{
		TypeInt, TypeFloat, TypeMessage
	};

	struct SubMenu;

	struct Item{
		char* name = nullptr;
		string identity = "";
		MessageType type = MessageType::TypeInt;
		uint8_t message_index = 0;
		bool can_change = false;
		SubMenu* next_page = nullptr;
	};
	struct SubMenu{
		SubMenu* previous_page = nullptr;
		char* submenu_name = nullptr;
		string identity = "";
		vector<Item> submenu_items;
		uint8_t number = submenu_items.size();
	};

	SubMenu home_page;

	DualCarMenu(St7735r* lcd, LcdTypewriter* writer, uint8_t image_width, uint8_t image_height);

	void AddItem(char* input_name, int* input_data, SubMenu* under_menu, bool can_change);

	void AddItem(char* input_name, float* input_data, SubMenu* under_menu, bool can_change);

	void AddItem(char* input_name, SubMenu* under_menu, bool HavSub);

	void PrintItem(Item item, uint8_t row, bool isSelected = false);

	SubMenu* PrintSubMenu(SubMenu* menu);//return the current page

	void PrintCamImage();

	void change_number();

	void SetCamBuffer(const Byte* camBuffer){
		this->camBuffer = camBuffer;
	}

	void SetEdge(vector<pair<int, int>> edge){
		this->edge = edge;
	}

	void SetJoystickState(Joystick::State state){
		joystick_state = state;
	}

private:
	vector<int*> int_data;
	vector<float*> float_data;
	St7735r* lcd;
	LcdTypewriter* writer;
	const Byte* camBuffer;
	uint8_t image_width;
	uint8_t image_height;
	Joystick::State joystick_state = Joystick::State::kIdle;
	uint8_t max_line = 10;
	int current_line = 0;
	int selected = false;
	bool pressed = false;
	vector<SubMenu*> memory;
	vector<pair<int, int>> edge;

	int8_t change_number_row = 0;
	int8_t change_number_column = 0;

//	char * change_number_getOpt(int8_t row, int8_t column, bool isSelected = false);
};





#endif /* INC_MENUV2_H_ */
