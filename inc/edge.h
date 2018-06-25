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

	vector<pair<int,int>> check_left_edge(const Byte* camBuffer);

	vector<pair<int,int>> check_right_edge(const Byte* camBuffer);

	vector<pair<int,int>> check_edge(const Byte* camBuffer);

private:
	vector<pair<int,int>> m_edge;
};


bool check_if_fail(int topline, int bottomline, vector<pair<int,int>> intput_vector);

vector<pair<int,int>>check_edge(const Byte* camBuffer);

bool check_left_edge(int topline, int bottomline, const Byte* camBuffer, vector<pair<int,int>> &edge_coord);

bool check_right_edge(int topline, int bottomline, const Byte* camBuffer, vector<pair<int,int>> &edge_coord);



#endif /* INC_EDGE_H_ */
