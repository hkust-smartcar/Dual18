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
	edge = 200, slope, corner, edge_size, edge_xmid, mpu, end, dotLine, unclear
};

class M_Bluetooth {
public:
	M_Bluetooth() :
			m_bt(
					myConfig::GetBluetoothConfig(
							std::function<
									bool(const Byte *data, const size_t size)>(
									[this](const Byte* buff, const size_t size)-> bool {
										change_message(buff);
										build_message(buff);

										return true;}))) {
	}
	;

	void change_message(const Byte* m_buff);

	void build_message(const Byte* m_buff);

	void set_edge(int size);

	void set_corner(int size);

	vector<pair<int, int>> get_m_edge() {
		return m_edge;
	}

	float get_m_slope() {
		return m_slope;
	}

	vector<Corner> get_slave_corner() {
		return slave_corner;
	}

	int get_edge_size() {
		return edge_size;
	}
	int get_edge_xmid() {
		return edge_mid;
	}
	int get_mpu_data() {
		return mpu_data;
	}
	int get_dotLine_data() {
		return dotLine;
	}

private:
	JyMcuBt106 m_bt;
	vector<Byte> buffer;
	int information_types = Informations::unclear;
	int edge_mid= 0;
	int edge_size = 0;
	int mpu_data = 0;
	int dotLine = 0;
	float m_slope = 0;
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
	void send_slope(float m_slope);
	void send_int_data(int edge_size, int type);
	void send_corner(vector<Corner> slave_corner);
private:
	JyMcuBt106 m_bt;

};

#endif /* INC_BLUETOOTH_H_ */
