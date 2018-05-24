/*
 * bluetooth.cpp
 *
 *  Created on: Mar 9, 2018
 *      Author: morristseng
 */
#include "bluetooth.h"
#include <vector>

void M_Bluetooth::set_y_coord(){
	m_edge.clear();
	for (int i=3; i<2*how_many_lines+3; i+=2){
		m_edge.push_back(std::make_pair(buffer[i], buffer[i+1]));
	}
	buffer.clear();
}
void S_Bluetooth::send_edge(std::vector<std::pair<int,int>> input_vector){
	int how_many_lines = input_vector.size();
	int size = 2*how_many_lines+4;
	Byte *buffer = new Byte[size];
	buffer[0] = Informations::edge;
	buffer[1] = size;
	buffer[2] =how_many_lines;
	buffer[size-1] = Informations::end;

	for(int i=0; i<how_many_lines; i++){
		buffer[3+2*i] = input_vector[i].first;
		buffer[4+2*i] = input_vector[i].second;
	}

	m_bt.SendBuffer(buffer, size);
	delete buffer;
}

void S_Bluetooth::send_info(bool fail_or_not){
	int size = 4;
	Byte *buffer = new Byte[size];
	buffer[0] = Informations::fail_on_turn;
	buffer[1] = size;
	buffer[2] = fail_or_not;
	buffer[3] = Informations::end;

	m_bt.SendBuffer(buffer,size);
	delete buffer;
}
