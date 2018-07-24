/*
 * edge.h
 *
 *  Created on: Feb 23, 2018
 *      Author: morristseng
 */

#ifndef INC_EDGE_H_
#define INC_EDGE_H_

#include <iostream>
#include "config.h"
#include <libsc/k60/ov7725.h>
#include <libsc/k60/ov7725_configurator.h>
#include <libsc/led.h>
#include <libsc/lcd_typewriter.h>
#include <libsc/st7735r.h>
#include <libsc/lcd.h>
#include <vector>
using namespace std;

class Edge{
public:
	enum Direction{
		Up, UpRight, Right, DownRight, Down, DownLeft, Left, UpLeft
	};


	Edge(bool clockwise, uint8_t topline, uint8_t bottomline):type(clockwise), topline(topline), bottomline(bottomline){
		lines = bottomline-topline;
		for(int i = 0; i < 80; ++i) {
		    canvas[i] = new uint8_t[lines];
		}
		for(int j=0; j<lines; j++){
			for(int i=0; i<80; i++){
				canvas[i][j] = 0;
			}
		}

	}

	void traveling_left(uint8_t xcoord, uint8_t ycoord, uint8_t last_direction_from, const Byte* camBuffer);

	void traveling_right(uint8_t xcoord, uint8_t ycoord, uint8_t last_direction_from, const Byte* camBuffer);


	vector<pair<int,int>>check_edge(const Byte* camBuffer);

	uint8_t check_junctions(uint8_t ycoord){
		uint8_t junctions = 0;
		if(m_edge.size()==0){
			return junctions;
		}
		for(int i= 0; i<m_edge.size(); i++){
			if((m_edge[i].second == ycoord)&&(abs(m_edge[i].first-junction_arry[ycoord-topline])>5)){
				junction_arry[ycoord-topline] = m_edge[i].first;
				junctions++;
			}
		}
		return junctions;
	}

	void reset_canvas(){
		for(int j=0; j<lines; j++){
			for(int i=0; i<80; i++){
				canvas[i][j] = 0;
			}
		}
	}

	void reset_junction_arry(){
		for(int i=0; i<lines; i++){
			junction_arry[i] = 0;
		}
	}

private:
	const uint8_t topline;
	const uint8_t bottomline;
	uint8_t lines;
	uint8_t** canvas = new uint8_t*[80];
	int* junction_arry = new int[lines];
	bool type;
	bool fail = false;
	vector<pair<int,int>> m_edge;

};




#endif /* INC_EDGE_H_ */
