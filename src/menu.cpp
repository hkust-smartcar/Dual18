/*
 * menu.cpp
 *
 *  Created on: Jun 21, 2018
 *      Author: morristseng
 */


#include "menu.h"

using namespace std;

void DualCar_Menu::change_mode(bool left){
	selected = false;
	line = 1;
	if(left){
		mode ++;
		changed = true;
	}
	else{
		if(mode!=0){
			mode--;
			changed = true;
		}
	}

}

bool DualCar_Menu::change_screen(){
	bool clear_screen = false;
	if(changed){
		changed = false;
		clear_screen = true;
	}
	return clear_screen;
}

void DualCar_Menu::select_pressed(){
	selected = !selected;//true==selected, false==not selected
	return;
}

void DualCar_Menu::change_line(bool up){// true == up, false == down
	if(!selected){
		if(up){
			if(line!=1)
				line--;
		}
		else{
			if(line!=m_menu[mode]->get_max_line())
				line++;
		}
	}
	else{
		if(m_menu[mode]->m_items[line-1]->get_if_can_change()){
			if(up){
				m_menu[mode]->m_items[line-1]->change_float_data(true);
			}
			else{
				m_menu[mode]->m_items[line-1]->change_float_data(false);
			}
		}
	}
	return;
}










