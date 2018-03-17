/*
 * bluetooth.cpp
 *
 *  Created on: Mar 9, 2018
 *      Author: morristseng
 */
#include "bluetooth.h"
#include <vector>

void M_Bluetooth::set_y_coord(){
	int t = buffer.size();
	for (int i=1; i<2*how_many_lines+1;){
		m_edge.push_back(std::make_pair(buffer[i], buffer[i+1]));
//		m_edge.emplace_back
		i += 2;
	}


	buffer.clear();
}

void S_Bluetooth::send_edge(std::vector<std::pair<int,int>> input_vector){
	int how_many_lines = input_vector.size();
	Byte *buffer = new Byte[2*how_many_lines+1];
	buffer[0] = how_many_lines;
	for(int i=0; i<how_many_lines; i++){
		*(buffer+(1+2*i)) = (input_vector[i].first);
		*(buffer+(2+2*i)) = (input_vector[i].second);
		if(i==(how_many_lines-1)){
			m_line = *(buffer+(2+2*i));
		}
	}
//	m_line = *(buffer);

	m_bt.SendBuffer(buffer, 2*how_many_lines+1);
	delete buffer;

}
