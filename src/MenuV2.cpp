/*
 * MenuV2.cpp
 *
 *  Created on: Jul 9, 2018
 *      Author: morristseng
 */

#include "MenuV2.h"

DualCarMenu::DualCarMenu(St7735r* lcd, LcdTypewriter* writer, uint8_t image_width, uint8_t image_height) {
	this->lcd = lcd;
	this->writer = writer;
	this->camBuffer = nullptr;
	this->image_width = image_width;
	this->image_height = image_height;
	home_page.identity = "home_page";

	change_number_newValue = (char*) malloc(20);
}

void DualCarMenu::AddItem(char* input_name, float* input_data, SubMenu* under_menu, bool can_change) {
	if (input_data == nullptr)
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
	if ((can_change) && (under_menu != nullptr)) {
		item.next_page = new SubMenu;
		item.next_page->identity = "changed";
	}
	under_menu->submenu_items.push_back(item);
	return;
}

void DualCarMenu::AddItem(char* input_name, int* input_data, SubMenu* under_menu, bool can_change) {
	if (input_data == nullptr)
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
	if ((can_change) && (under_menu != nullptr)) {
		item.next_page = new SubMenu;
		item.next_page->identity = "changed";
	}
	under_menu->submenu_items.push_back(item);
	return;
}

void DualCarMenu::AddItem(char* input_name, SubMenu* under_menu, bool HavSub) {
	Item item;
	item.name = input_name;
	string m_identity(input_name);
	item.identity = m_identity;
	item.type = MessageType::TypeMessage;
	item.next_page = nullptr;
	if ((HavSub) && (under_menu != nullptr)) {
		item.next_page = new SubMenu;
		item.next_page->identity = m_identity;
	}
	under_menu->submenu_items.push_back(item);
	return;
}

void DualCarMenu::PrintItem(Item item, uint8_t row, bool isSelected) {
	char c[20];
	for (int i = 0; i < 20; i++) {
		c[i] = ' ';
	}
	lcd->SetRegion(Lcd::Rect(0, 15 * row, 120, 15));
	switch (item.type) {
		case MessageType::TypeInt:
			sprintf(c, " %s: %d", item.name, *int_data[item.message_index]);
			break;
		case MessageType::TypeFloat:
			sprintf(c, " %s: %.3f", item.name, *float_data[item.message_index]);
			break;
		case MessageType::TypeMessage:
			sprintf(c, " %s", item.name);
			break;
		default:
			sprintf(c, " null");
			break;
	}
	c[0] = isSelected ? '>' : ' ';
	writer->WriteBuffer(c, 20);
}

DualCarMenu::SubMenu* DualCarMenu::PrintSubMenu(SubMenu* menu) {

	uint8_t change_number_action; // dummy read
	change_number_getOpt(change_number_row, change_number_column, change_number_action);

	AddItem((char*) "exit", menu, false);
	switch (joystick_state) {
		case Joystick::State::kSelect:
			if (menu->identity == "changed") {
				if (strlen(change_number_newValue) < 15) {
					char c[1];
					if (change_number_action < 10) {
						// 1 to 9
						sprintf(c, "%d", change_number_action);
						strcat(change_number_newValue, c);
					} else if (change_number_action == 10 && strlen(change_number_newValue) != 0) {
						// del - remove last character
						change_number_newValue[strlen(change_number_newValue) - 1] = 0;
					} else if (change_number_action == 11) {
						// 0
						sprintf(c, "%d", 0);
						strcat(change_number_newValue, c);
					} else if (change_number_action == 12 && strlen(change_number_newValue) != 0) {
						// .
						sprintf(c, ".");
						strcat(change_number_newValue, c);
					} else if (change_number_action == 13) {
						// back
						if (selected > 0)
							selected = 0;
						else
							selected = 1;
						pressed = !pressed;
					} else if (change_number_action == 14) {
						// save
						float t = 7777;
//						std::sscanf(change_number_newValue, "%f", &t);
						t = atof(change_number_newValue);
						if (change_number_item_ptr->type == MessageType::TypeInt) {
							*int_data[change_number_item_ptr->message_index] = (int) t;
						} else {
							*float_data[change_number_item_ptr->message_index] = t;
						}

						if (selected > 0)
							selected = 0;
						else
							selected = 1;
						pressed = !pressed;
					} else if (change_number_action == 15) {
						// clear
						sprintf(change_number_newValue, "");
					}
//					strcat(change_number_newValue, (char*) (change_number_action+48)); // ascii conversion
				}
			} else {
				if (selected > 0)
					selected = 0;
				else
					selected = 1;
				pressed = !pressed;

			}
			joystick_state = Joystick::State::kIdle;
			break;
		case Joystick::State::kUp:
//			if ((selected) && (menu->submenu_items[current_line].can_change)) {
//				if (menu->submenu_items[current_line].type == MessageType::TypeInt) {
//					(*int_data[menu->submenu_items[current_line].message_index])++;
//				}
//			} else if (!selected) {
			if (current_line > 0) {
				current_line--;
			} else {
				current_line = (menu->submenu_items.size() - 1);
			}
//			}
			if (menu->identity == "changed")
				change_number_row--;
			joystick_state = Joystick::State::kIdle;
			break;
		case Joystick::State::kDown:
//			if ((selected) && (menu->submenu_items[current_line].can_change)) {
//				if (menu->submenu_items[current_line].type == MessageType::TypeInt) {
//					if ((*int_data[menu->submenu_items[current_line].message_index]) > 0) {
//						(*int_data[menu->submenu_items[current_line].message_index])--;
//					}
//				}
//			} else if (!selected) {
			if (current_line < (menu->submenu_items.size() - 1)) {
				current_line++;
			} else {
				current_line = 0;
			}
//			}
			if (menu->identity == "changed")
				change_number_row++;
			joystick_state = Joystick::State::kIdle;
			break;
		case Joystick::State::kLeft:
			if (menu->identity == "changed")
				change_number_column--;
			joystick_state = Joystick::State::kIdle;
			break;
		case Joystick::State::kRight:
			if (menu->identity == "changed")
				change_number_column++;
			joystick_state = Joystick::State::kIdle;
			break;
		default:
			pressed = false;
			break;
	}

	if (menu->identity == "changed") {
		change_number_row = change_number_row == -1 ? 4 : change_number_row;
		change_number_row = change_number_row == 5 ? 0 : change_number_row;
		change_number_column = change_number_column == -1 ? 2 : change_number_column;
		change_number_column = change_number_column == 3 ? 0 : change_number_column;
	}

	if ((pressed) && (menu->submenu_items[current_line].next_page != nullptr)) {
		pressed = false;
		selected = false;

		// need better assignment for this VVV
		change_number_item_ptr = &menu->submenu_items[current_line];
		uint8_t temp = current_line;
		menu->submenu_items[temp].next_page->previous_line= current_line;
		current_line = 0;
		menu->submenu_items.erase(menu->submenu_items.end());
		menu->submenu_items[menu->submenu_items[temp].next_page->previous_line].next_page->previous_page = menu;
		lcd->Clear();
		return menu->submenu_items[menu->submenu_items[temp].next_page->previous_line].next_page;
	} else if ((pressed) && (menu->submenu_items[current_line].identity == "exit")
			&& (menu->identity != "home_page")) {
		change_number_action = 0;
		pressed = false;
		selected = false;
		current_line = menu->previous_line;
		menu->submenu_items.erase(menu->submenu_items.end());
		lcd->Clear();
		return menu->previous_page;
	}

	if (menu->identity == "image") {
		PrintCamImage();
		for (uint16_t i = 0; i < (menu->submenu_items.size()) && (i < max_line - 4); i++) {
			PrintItem(menu->submenu_items[i], i + 4);
		}
	} else if ((menu->identity == "OpenMotor") || (menu->identity == "CloseMotor")) {
	} else if (menu->identity == "changed") {
		if (!change_number_isOn) {
			// init the value
			char c[20];
			if (change_number_item_ptr->type == MessageType::TypeInt) {
				sprintf(c, "%d", *int_data[change_number_item_ptr->message_index]);
			} else {
				sprintf(c, "%.3f", *float_data[change_number_item_ptr->message_index]);
			}
			strcpy(change_number_newValue, c);
			lcd->Clear();
			change_number_isOn = true;

			change_number_row = 3;
			change_number_column = 1;
		}

		char c[20];
		lcd->SetRegion(Lcd::Rect(0, 15 * 0, 120, 15));
		if (change_number_item_ptr->type == MessageType::TypeInt) {
			sprintf(c, "*%s: %d                ", change_number_item_ptr->name,
					*int_data[change_number_item_ptr->message_index]);
		} else {
			sprintf(c, "*%s: %.3f                ", change_number_item_ptr->name,
					*float_data[change_number_item_ptr->message_index]);
		}
		writer->WriteBuffer(c, 20);

		lcd->SetRegion(Lcd::Rect(10, 15 * 1, 120, 15));
		strcpy(c, change_number_newValue);
		strcat(c, (char *) "                     ");
		writer->WriteBuffer(c, 20);

		for (uint8_t y = 0; y < 5; y++) {
			char* c = (char*) malloc(30);
			uint8_t change_number_action; // dummy read
			strcpy(c, change_number_getOpt(y, 0, change_number_action));
			strcat(c, change_number_getOpt(y, 1, change_number_action));
			strcat(c, change_number_getOpt(y, 2, change_number_action));
			lcd->SetRegion(Lcd::Rect(0, 15 * (y + 3), 120, 15));
			writer->WriteBuffer(c, 30);
			free(c);
		}
	} else {
		for (uint16_t i = 0; i < (menu->submenu_items.size()) && (i < max_line); i++) {
			PrintItem(menu->submenu_items[i], i, current_line == i);
		}
	}
	menu->submenu_items.erase(menu->submenu_items.end());

	change_number_isOn = menu->identity == "changed";
	return menu;

}

void DualCarMenu::PrintCamImage() {
	lcd->SetRegion(Lcd::Rect(0, 0, image_width, image_height));
	lcd->FillBits(0x0000, 0xFFFF, camBuffer, (image_width * image_height));

	for (uint16_t i = 0; i < edge.size(); i++) {
		lcd->SetRegion(Lcd::Rect(edge[i].first, edge[i].second, 2, 2));
		lcd->FillColor(Lcd::kRed);
	}

	for (uint16_t i = 0; i < corner.size(); i++) {
		lcd->SetRegion(Lcd::Rect(corner[i].get_xcoord(), corner[i].get_ycoord(), 2, 2));
		lcd->FillColor(Lcd::kBlue);
	}

}

inline char * DualCarMenu::change_number_getOpt(const int8_t &row, const int8_t &column, uint8_t &action) {
	char str[5];
	bool isSelected = change_number_row == row && change_number_column == column;
	char selectLeft = isSelected ? '>' : ' ';
	char selectRight = isSelected ? '<' : ' ';
	switch (row) {
		case 0:
		case 1:
		case 2:
			action = (row * 3 + (column + 1));
			sprintf(str, "%c %d %c", selectLeft, action, selectRight);
			break;
		case 3: {
			switch (column) {
				case 0: {
					// del
					action = 10;
					sprintf(str, "%c%s%c", selectLeft, "Del", selectRight);
				}
					break;
				case 1: {
					// 0
					action = 11;
					sprintf(str, "%c%s%c", selectLeft, " 0 ", selectRight);
				}
					break;
				case 2: {
					// .
					action = 12;
					sprintf(str, "%c%s%c", selectLeft, " . ", selectRight);
				}
					break;
				default:
					break;
			}
		}
			break;
		case 4: {
			switch (column) {
				case 0: {
					action = 13;
					sprintf(str, "%c%s%c", selectLeft, "BK ", selectRight);
					// Back
				}
					break;
				case 1: {
					action = 14;
					sprintf(str, "%c%s%c", selectLeft, "SAV", selectRight);
					// Enter
				}
					break;
				case 2: {
					action = 15;
					sprintf(str, "%c%s%c", selectLeft, "Clr", selectRight);
					// Clear
				}
					break;
				default:
					break;
			}
		}
			break;
		default:
			break;
	}

	return str;
}
