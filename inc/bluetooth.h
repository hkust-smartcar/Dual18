/*
 * bluetooth.h
 *
 *  Created on: Mar 9, 2018
 *      Author: morristseng
 */

#ifndef INC_BLUETOOTH_H_
#define INC_BLUETOOTH_H_

#include "config.h"
#include <vector>
#include <functional>
#include <libbase/k60/mcg.h>
#include <libsc/system.h>
#include "libsc/k60/jy_mcu_bt_106.h"
#include "libbase/k60/pit.h"
#include "libsc/st7735r.h"
#include "libsc/lcd_typewriter.h"
#include "cstring"
#include "corner.h"

using libsc::k60::JyMcuBt106;
using libbase::k60::Pit;
using std::vector;
using std::function;
using libsc::System;
using namespace std;

enum Informations {
	edge = 100, fail_on_turn, corner, end, unclear
};

//class Package {
//	Package(Informations type, vector<uint8_t> data) {
//		this->type = type;
//		this->data = data;
//	}
//	Package(Informations type, vector<pair<uint8_t, uint8_t>> coordinate) {
//		this->type = type;
//		for (int i = 0; i < coordinate.size(); i++) {
//			this->data.push_back(coordinate[i].first);
//			this->data.push_back(coordinate[i].second);
//		}
//	}
//
//private:
//	Informations type;
//	vector<uint8_t> data;
//
//};

class M_Bluetooth {
public:
	M_Bluetooth() :
			m_bt(
					myConfig::GetBluetoothConfig(
							std::function<
									bool(const Byte *data, const size_t size)>(
									[this](const Byte* buff, const size_t size)-> bool {
										if(((*buff)==Informations::edge)&&(buffer.size()==0)) {
											buffer.clear();
											information_types = Informations::edge;
										}
										if((*buff==Informations::fail_on_turn)&&(buffer.size()==0)) {
											buffer.clear();
											information_types = Informations::fail_on_turn;
										}
										if((*buff==Informations::corner)&&(buffer.size()==0)) {
											buffer.clear();
											information_types = Informations::corner;
										}

										if(information_types == Informations::edge) {
											this->buffer.push_back(*buff);
											if(((*buff)==Informations::end)) {
												int size = buffer[1];
												if(size!=0) {
													how_many_lines = buffer[2];
													if(buffer.size() == size) {
														reset_m_edge();
														set_y_coord();
														buffer.clear();
													}
													else{
														reset_m_edge();
														buffer.clear();
													}
												}
												else {
													reset_m_edge();
													buffer.clear();
												}
											}
										}

										else if((information_types == Informations::fail_on_turn)) {
											this->buffer.push_back(*buff);
											if(((*buff)==Informations::end)) {
												fail_on_turn = buffer[2];
												buffer.clear();
											}
											else{
												buffer.clear();
											}
										}

										else if(information_types == Informations::corner) {
											this->buffer.push_back(*buff);

											int temp = 0;
											temp = *buff;
											reset_slave_corner();
											if(((*buff)==Informations::end)) {
												int size = 3*buffer[1]+3;
												if(size!=3) {
													corner_size = buffer[1];
													if(buffer.size() == size) {
														reset_slave_corner();
														set_corner();
														buffer.clear();
													}
													else{
														reset_slave_corner();
														buffer.clear();
													}
												}
												else {
													reset_slave_corner();
													buffer.clear();
												}
											}

										}

										else {
											buffer.clear();
										}

										return true;}))) {
	}
	;

	void set_y_coord();

	void set_corner();

	int get_how_many_line() {
		return how_many_lines;
	}

	vector<pair<int, int>> get_m_edge() {
		return m_edge;
	}

	bool get_fail_on_turn() {
		return fail_on_turn;
	}

	vector<Corner> get_slave_corner() {
		return slave_corner;
	}

	int get_corner_size(){return corner_size;}

	void reset_m_edge() {
		m_edge.clear();
	}

	void reset_slave_corner() {
		slave_corner.clear();
	}

private:
	JyMcuBt106 m_bt;
	vector<Byte> buffer;
	int information_types = Informations::unclear;
	int how_many_lines = 0;
	int corner_size = 0
	bool fail_on_turn = 0;
	std::vector<int> y_coord();
	std::vector<std::pair<int, int>> m_edge;
	vector<Corner> slave_corner;
};

class S_Bluetooth {
public:
	S_Bluetooth() :
			m_bt(myConfig::GetSlaveBluetoothConfig()) {
	}
	;

	void send_edge(std::vector<std::pair<int, int>> input_vector);
	void send_info(bool fail_or_not);
	void send_corner(vector<Corner> slave_corner);
private:
	JyMcuBt106 m_bt;

};

#endif /* INC_BLUETOOTH_H_ */
