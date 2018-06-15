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


using libsc::k60::JyMcuBt106;
using libbase::k60::Pit;
using std::vector;
using std::function;
using libsc::System;

class M_Bluetooth{
public:
	M_Bluetooth():m_bt(myConfig::GetBluetoothConfig(std::function<bool(const Byte *data, const size_t size)>([this](const Byte* buff, const size_t size)-> bool{
		this->buffer.push_back(*buff);
		how_many_lines = buffer[0];
		if(buffer.size() == 2*how_many_lines+1){
			reset_m_edge();
			this->set_y_coord();
//			buffer.clear();
		}

    	return true;})))
{};

	void set_y_coord();

	std::vector<std::pair<int,int>> get_m_edge(){ return m_edge;}

	void reset_m_edge(){ m_edge.clear();}

	std::vector<Byte> buffer;

private:
	JyMcuBt106 m_bt;
	int how_many_lines;
	std::vector<int> y_coord();
	std::vector<std::pair<int,int>> m_edge;
};

class S_Bluetooth{
public:
	S_Bluetooth():
	m_bt(myConfig::GetSlaveBluetoothConfig())
{};

	void send_edge(std::vector<std::pair<int,int>> input_vector);
	int get_how_many_lines(){
		return m_line;
	}


private:
	JyMcuBt106 m_bt;
	int m_line;

};



#endif /* INC_BLUETOOTH_H_ */
