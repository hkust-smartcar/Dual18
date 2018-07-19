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

	Edge(bool clockwise):type(clockwise){
		for(int j=0; j<30; j++){
			for(int i=0; i<80; i++){
				traveled[i][j]=0;
			}
		}
	}

	void traveling_left(uint8_t xcoord, uint8_t ycoord, uint8_t last_direction_from, int topline, int bottomline, const Byte* camBuffer);

	void traveling_right(uint8_t xcoord, uint8_t ycoord, uint8_t last_direction_from, int topline, int bottomline, const Byte* camBuffer);


	vector<pair<int,int>>check_edge(const Byte* camBuffer, int topline, int bottomline);

	void reset_traveled(){
		for(int j=0; j<30; j++){
			for(int i=0; i<80; i++){
				traveled[i][j]=0;
			}
		}
	}

private:
	bool traveled[80][30];//30 rolls
	bool type;
	vector<pair<int,int>> m_edge;
};




#endif /* INC_EDGE_H_ */
