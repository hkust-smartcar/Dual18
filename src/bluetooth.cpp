/*
 * bluetooth.cpp
 *
 *  Created on: Mar 9, 2018
 *      Author: morristseng
 */
#include "bluetooth.h"
#include "corner.h"
#include <vector>

void M_Bluetooth::set_y_coord(){
	m_edge.clear();
	for (int i=3; i<2*how_many_lines+3; i+=2){
		m_edge.push_back(std::make_pair(buffer[i], buffer[i+1]));
	}
	buffer.clear();
}

void M_Bluetooth::set_corner(){
	slave_corner.clear();
	for(int i=2; i<3*buffer[1]; i+=3){
		int x_coord = buffer[i];
		int y_coord = buffer[i+1];
		float percentage = buffer[i+2]/100.0;
		Corner temp(x_coord, y_coord, percentage);
		slave_corner.push_back(temp);
	}
	buffer.clear();
}

void S_Bluetooth::send_edge(std::vector<std::pair<int,int>> slave_edge){
	if(slave_edge.size()==0){
		Byte *buffer = new Byte[3];
		buffer[0] = Informations::edge;
		buffer[1] = 0;//size
		buffer[2] = Informations::end;
		m_bt.SendBuffer(buffer,3);
		delete buffer;
	}
	if(slave_edge.size()>0){
		int how_many_lines = slave_edge.size();
		int size = 2*how_many_lines+4;
		Byte *buffer = new Byte[size];
		buffer[0] = Informations::edge;
		buffer[1] = size;
		buffer[2] =how_many_lines;
		buffer[size-1] = Informations::end;

		for(int i=0; i<how_many_lines; i++){
			buffer[3+2*i] = slave_edge[i].first;
			buffer[4+2*i] = slave_edge[i].second;
		}

		m_bt.SendBuffer(buffer, size);
		delete []buffer;
	}
}

void S_Bluetooth::send_info(bool fail_or_not){
	int size = 4;
	Byte *buffer = new Byte[size];
	buffer[0] = Informations::fail_on_turn;
	buffer[1] = size;
	buffer[2] = fail_or_not;
	buffer[3] = Informations::end;

	m_bt.SendBuffer(buffer,size);
	delete []buffer;
}

void S_Bluetooth::send_corner(vector<Corner> slave_corner){
	if(slave_corner.size()==0){
		Byte *buffer = new Byte[3];
		buffer[0] = Informations::corner;
		buffer[1] = 0;
		buffer[2] = Informations::end;
		m_bt.SendBuffer(buffer,3);
		delete []buffer;
	}
	if(slave_corner.size()>0){
		int size = 3*slave_corner.size()+3;
		int num_of_corner = slave_corner.size();
		Byte *buffer = new Byte[size];
		buffer[0] = Informations::corner;
		buffer[1] = num_of_corner;
		buffer[size-1] = Informations::end;
		for(int i=0; i<num_of_corner; i++){
			buffer[2+3*i] = slave_corner[i].get_xcoord();
			buffer[3+3*i] = slave_corner[i].get_ycoord();
			buffer[4+3*i] = (uint8_t)(slave_corner[i].get_percentage())*100;

		}

		m_bt.SendBuffer(buffer,size);
		delete []buffer;
	}
}
