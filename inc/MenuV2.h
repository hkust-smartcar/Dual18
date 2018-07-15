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
#include "corner.h"
using namespace std;

class DualCarMenu {
public:
	enum MessageType {
		TypeInt, TypeFloat, TypeMessage
	};

	struct SubMenu;

	struct Item {
		char* name = nullptr;
		string identity = "";
		MessageType type = MessageType::TypeInt;
		uint8_t message_index = 0;
		bool can_change = false;
		SubMenu* next_page = nullptr;
	};
	struct SubMenu {
		SubMenu* previous_page = nullptr;
		uint8_t previous_line = 0;
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

	SubMenu* PrintSubMenu(SubMenu* menu); //return the current page

	void PrintCamImage();

	void SetCamBuffer(const Byte* camBuffer) {
		this->camBuffer = camBuffer;
	}

	void SetEdge(vector<pair<int, int>> edge) {
		this->edge = edge;
	}

	void SetCorner(vector<Corner> corner) {
		this->corner = corner;
	}

	void SetJoystickState(Joystick::State state) {
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
	uint8_t current_line = 0;
	int selected = false;
	bool pressed = false;
	vector<SubMenu*> memory;
	vector<pair<int, int>> edge;
	vector<Corner> corner;

	int8_t change_number_row = 0;
	int8_t change_number_column = 0;
	Item *change_number_item_ptr = nullptr;
	bool change_number_isOn = false;
	char *change_number_newValue;

	char * change_number_getOpt(const int8_t &row, const int8_t &column, uint8_t &action);
};

#endif /* INC_MENUV2_H_ */
