/*
 * MenuV2.cpp
 *
 *  Created on: Jul 9, 2018
 *      Author: morristseng
 */

#include "MenuV2.h"

DualCarMenu::DualCarMenu(St7735r* lcd, LcdTypewriter* writer, uint8_t image_width, uint8_t image_height){
	this->lcd = lcd;
	this->writer = writer;
	this->camBuffer = nullptr;
	this->image_width = image_width;
	this->image_height = image_height;
	home_page.identity = "home_page";
}

void DualCarMenu::change_number(){
	return;
}

void DualCarMenu::AddItem(char* input_name, float* input_data, SubMenu* under_menu, bool can_change){
	if(input_data == nullptr)
		return;
	Item item;
	item.name = input_name;
	string m_identity(input_name);
	item.identity = m_identity;
	item.type = MessageType::TypeFloat;
	item.next_page = nullptr;
	item.can_change = can_change;
	item.message_index = float_data.size();
	float_data.push_back(input_data);
	if((can_change)&&(under_menu!=nullptr)){
		item.next_page = new SubMenu;
		item.next_page->identity = "changed";
	}
	under_menu->submenu_items.push_back(item);
	return;
}


void DualCarMenu::AddItem(char* input_name, int* input_data, SubMenu* under_menu, bool can_change){
	if(input_data == nullptr)
		return;
	Item item;
	item.name = input_name;
	string m_identity(input_name);
	item.identity = m_identity;
	item.type = MessageType::TypeInt;
	item.next_page = nullptr;
	item.can_change = can_change;
	item.message_index = int_data.size();
	int_data.push_back(input_data);
	if((can_change)&&(under_menu!=nullptr)){
		item.next_page = new SubMenu;
		item.next_page->identity = "changed";
	}
	under_menu->submenu_items.push_back(item);
	return;
}

void DualCarMenu::AddItem(char* input_name, SubMenu* under_menu, bool HavSub){
	Item item;
	item.name = input_name;
	string m_identity(input_name);
	item.identity = m_identity;
	item.type = MessageType::TypeMessage;
	item.next_page = nullptr;
	if((HavSub)&&(under_menu!=nullptr)){
		item.next_page = new SubMenu;
		item.next_page->identity = m_identity;
	}
	under_menu->submenu_items.push_back(item);
	return;
}

void DualCarMenu::PrintItem(Item item, uint8_t row){
	char c[15];
	for(int i=0; i<15; i++){
		c[i] = ' ';
	}
	lcd->SetRegion(Lcd::Rect(0, 15*row, 80, 15));
	switch(item.type){
		case MessageType::TypeInt:
			sprintf(c, "%s: %d", item.name, *int_data[item.message_index]);
			writer->WriteBuffer(c,15);
			break;
		case MessageType::TypeFloat:
			sprintf(c, "%s: %.3float", item.name, *float_data[item.message_index]);
			writer->WriteBuffer(c,15);
			break;
		case MessageType::TypeMessage:
			sprintf(c, "%s", item.name);
			writer->WriteBuffer(c,15);
			break;
		default:
			break;
	}
}

DualCarMenu::SubMenu* DualCarMenu::PrintSubMenu(SubMenu* menu){


	AddItem("exit", menu, false);
	switch(joystick_state){
		case Joystick::State::kSelect:
			if(selected>0)
				selected = 0;
			else
				selected = 1;
			pressed = !pressed;
			joystick_state = Joystick::State::kIdle;
			break;
		case Joystick::State::kUp:
			if((selected)&&(menu->submenu_items[current_line].can_change)){
				if(menu->submenu_items[current_line].type == MessageType::TypeInt){
					(*int_data[menu->submenu_items[current_line].message_index])++;
				}
			}
			else if(!selected){
				if(current_line>0)
					current_line--;
			}

			joystick_state = Joystick::State::kIdle;
			break;
		case Joystick::State::kDown:
			if((selected)&&(menu->submenu_items[current_line].can_change)){
				if(menu->submenu_items[current_line].type == MessageType::TypeInt){
					if((*int_data[menu->submenu_items[current_line].message_index])>0){
						(*int_data[menu->submenu_items[current_line].message_index])--;
					}
				}
			}
			else if(!selected){
				if(current_line<(menu->submenu_items.size()-1))
					current_line++;
			}
			joystick_state = Joystick::State::kIdle;
			break;
		case Joystick::State::kLeft:
			joystick_state = Joystick::State::kIdle;
			break;
		case Joystick::State::kRight:
			joystick_state = Joystick::State::kIdle;
			break;
		default:
			pressed = false;
			break;
	}

	if((pressed)&&(menu->submenu_items[current_line].next_page!=nullptr)){
		pressed = false;
		selected = false;
		uint8_t temp = current_line;
		current_line = 0;
		menu->submenu_items.erase(menu->submenu_items.end());
		menu->submenu_items[temp].next_page->previous_page = menu;
		lcd->Clear();
		return menu->submenu_items[temp].next_page;
	}
	else if((pressed)&&(menu->submenu_items[current_line].identity == "exit")&&(menu->identity!="home_page")){
		pressed = false;
		selected = false;
		uint8_t temp = current_line;
		current_line = 0;
		menu->submenu_items.erase(menu->submenu_items.end());
		lcd->Clear();
		return menu->previous_page;
	}


	if(selected)
		AddItem("selected", menu, false);
	else
		AddItem("unselected", menu, false);
	AddItem("line", &current_line, menu, false);

	if(menu->identity == "image"){
		PrintCamImage();
		for(int i=0; i<(menu->submenu_items.size())&&(i<max_line-4); i++){
			PrintItem(menu->submenu_items[i],i+4);
		}
	}
	else if((menu->identity == "OpenMotor")||(menu->identity == "CloseMotor")){}
	else{
		if(menu->identity == "changed"){
			change_number();
		}
		for(int i=0; i<(menu->submenu_items.size())&&(i<max_line); i++){
			PrintItem(menu->submenu_items[i],i);
		}
	}
	menu->submenu_items.erase(menu->submenu_items.end());
	menu->submenu_items.erase(menu->submenu_items.end());
	menu->submenu_items.erase(menu->submenu_items.end());

	return menu;

}

void DualCarMenu::PrintCamImage(){
	lcd->SetRegion(Lcd::Rect(0, 0, image_width, image_height));
	lcd->FillBits(0x0000, 0xFFFF, camBuffer, (image_width*image_height));

	for(int i=0; i<edge.size(); i++){
		lcd->SetRegion(Lcd::Rect(edge[i].first, edge[i].second, 2, 2));
		lcd->FillColor(Lcd::kRed);
	}

}
