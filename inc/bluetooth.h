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


using libsc::k60::JyMcuBt106;
using libbase::k60::Pit;
using std::vector;
using std::function;
using libsc::System;
using namespace std;

enum Informations{
	edge = 100,
	fail_on_turn,
	end
};

class Package{
	Package(Informations type, vector<uint8_t> data){
		this->type = type;
		this->data = data;
	}
	Package(Informations type, vector<pair<uint8_t,uint8_t>> coordinate){
		this->type = type;
		for(int i=0; i<coordinate.size();i++){
			this->data.push_back(coordinate[i].first);
			this->data.push_back(coordinate[i].second);
		}
	}

private:
	Informations type;
	vector<uint8_t> data;
};

//class Bluetooth{
//	 Bluetooth(): m_bt(myConfig::GetBluetoothConfig([this](const Byte* data, const size_t size){return handle_package(data, size);})) {}
//
//	 void build_package();
//
////	 void send_package(Package m_package) = 0;
//
//	 bool handle_package(const Byte* data, const size_t size){
//		 return true;
//	 }
//
//private:
//    JyMcuBt106 m_bt;
//
//};

class M_Bluetooth{
public:
	M_Bluetooth():m_bt(myConfig::GetBluetoothConfig(std::function<bool(const Byte *data, const size_t size)>([this](const Byte* buff, const size_t size)-> bool{
		if(((*buff)==Informations::edge)&&(buffer.size()==0)){
			buffer.clear();
			information_types = Informations::edge;

		}
		if((*buff==Informations::fail_on_turn)&&(buffer.size()==0)){
			buffer.clear();
			information_types = Informations::fail_on_turn;
		}

		if(information_types == Informations::edge){
			this->buffer.push_back(*buff);
			if(((*buff)==Informations::end)&&buffer.size()==buffer[1]){
				int size = buffer[1];
				how_many_lines = buffer[2];
				if(buffer.size() == size){
					set_y_coord();
					buffer.clear();
				}
			}
		}

		if((information_types == Informations::fail_on_turn)){
			this->buffer.push_back(*buff);
			if(((*buff)==Informations::end)&&buffer.size()==buffer[1]){
				fail_on_turn = buffer[2];
				buffer.clear();
			}
		}

    	return true;})))
{};
	std::vector<Byte> buffer;

	void set_y_coord();

	int get_how_many_line(){return how_many_lines;}

	std::vector<std::pair<int,int>> get_m_edge(){ return m_edge;}

	bool get_fail_on_turn(){return fail_on_turn; }

	void reset_m_edge(){ m_edge.clear();}

private:
	JyMcuBt106 m_bt;
	int information_types = Informations::edge;
	int how_many_lines = 0;
	bool fail_on_turn = 0;
	std::vector<int> y_coord();
	std::vector<std::pair<int,int>> m_edge;
};

class S_Bluetooth{
public:
	S_Bluetooth():
	m_bt(myConfig::GetSlaveBluetoothConfig())
{};

	void send_edge(std::vector<std::pair<int,int>> input_vector);
	void send_info(bool fail_or_not);


private:
	JyMcuBt106 m_bt;

};



#endif /* INC_BLUETOOTH_H_ */
