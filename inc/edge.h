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

vector<pair<int,int>>check_edge(const Byte* camBuffer);


#endif /* INC_EDGE_H_ */
