/*
 * corner.h
 *
 *  Created on: Feb 21, 2018
 *      Author: morristseng
 */

#ifndef INC_CORNER_H_
#define INC_CORNER_H_
#include <iostream>
#include "config.h"
#include <libsc/k60/ov7725.h>
#include <libsc/k60/ov7725_configurator.h>
#include <libsc/led.h>
#include <libsc/lcd_typewriter.h>
#include <libsc/st7735r.h>
#include <libsc/lcd.h>
#include <vector>
#include "edge.h"
using namespace std;

enum CornerDirection{
	Up, UpRight, Right, DownRight, Down, DownLeft, Left, UpLeft
};

class Corner{
public:
	Corner():percentage(0), x(0),y(0){}
	Corner(int m_x, int m_y, float m_percentage):percentage(m_percentage), x(m_x),y(m_y){}
	void set_percentage(float m_percentage){percentage = m_percentage;}
	void set_xcoord(int m_x){x = m_x;}
	void set_ycoord(int m_y){y = m_y;}

	float get_percentage(){return percentage;}
	int get_xcoord(){return x;}
	int get_ycoord(){return y;}

private:
	float percentage;
	int x;
	int y;
};
//uint8_t convolution(uint8_t xcoord, uint8_t ycoord, const Byte* camBuffer);

vector<pair<int,int>>check_corner_edge(const Byte* camBuffer, int topline, int bottomline);

Corner find_min(vector<Corner>);

vector<Corner>check_corner(const Byte* camBuffer, int topline, int bottomline, vector<pair<int,int>> edge);

float distance(int x1, int y1, int x2, int y2);

vector<Corner> check_cornerv2(const Byte* camBuffer, int topline, int bottomline, vector<pair<int,int>> edge);

#endif /* INC_CORNER_H_ */
