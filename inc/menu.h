/*
 * menu.h
 *
 *  Created on: Jun 21, 2018
 *      Author: morristseng
 */

#ifndef INC_MENU_H_
#define INC_MENU_H_

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

using namespace std;

class Items{
public:
	Items(char* input_message):const_float_data(0){
		strcpy(m_input_message,input_message);
		is_char =true;
		is_float = false;
		float_data = nullptr;
	}
	Items(char* input_message, float data):const_float_data(data){
		float_data = nullptr;
		strcpy(m_input_message,input_message);
		is_char =false;
		is_float = true;
	}
	Items(char* input_message, float* data, bool change):const_float_data(0){
		float_data = data;
		strcpy(m_input_message,input_message);
		is_char =false;
		is_float = true;
		can_change = change;
	}

	void change_value(float &temp){
		temp = temp+5;
	}

	void change_float_data(bool plus){
		if(plus)
			(*float_data) += increment;
		else
			(*float_data) -= increment;
	}

	char* get_message(){
		for(int i=0; i<15; i++){
			message[i] = ' ';
		}
		if((is_float)&&(float_data!=nullptr)){
			sprintf(message, "%s: %.2f", m_input_message, (*float_data));
		}
		else if(is_float){
			sprintf(message, "%s: %.2f", m_input_message, const_float_data);
		}
		else{
			sprintf(message, "%s",m_input_message);
		}
		return message;
	}

	void set_increment(float incre){
		increment = incre;
	}

	void set_float_data(float number){(*float_data) = number;}

	float get_float_data(){return *float_data;}

	const float get_const_float_data(){return const_float_data;}

	bool get_if_can_change(){return can_change;}

private:
	char m_input_message[15];
	char message[15];
	bool is_float;
	bool is_char;
	float* float_data;
	const float const_float_data;
	bool can_change = false;
	float increment = 0;
	float reference = 0;
};

class Mode{
public:
	Mode (uint8_t mode):m_mode(mode){}

	void add_items(Items* item){
		m_items.push_back(item);
		max_line = m_items.size();
	}

	uint8_t get_max_line(){return max_line;}

	vector<Items*> m_items;

	void clear(){m_items.clear();}

private:
	uint8_t m_mode;
	uint8_t max_line = 0;
};

class DualCar_Menu{
public:
	DualCar_Menu(){};

	void change_mode(bool);

	bool change_screen();

	void change_line(bool);

	void select_pressed();

	void add_mode(Mode* mode){
		m_menu.push_back(mode);
		mode_size = m_menu.size();
	}

	void create_menu(){

	}

	void clear(){
		m_menu.clear();
	}

	vector<Mode*> m_menu;

	uint8_t get_mode(){return mode;}

	bool get_selected(){return selected;}

	uint8_t get_line(){return line;}
	float get_temp(){return temp;}

private:
	uint8_t mode_size = 0;
	uint8_t mode = 0;
	uint8_t line = 1;
	bool changed = false;
	bool selected = false;
	float temp = 0;

};



#endif /* INC_MENU_H_ */
