/*
 * useful_functions.h
 *
 *  Created on: Apr 12, 2018
 *      Author: morristseng
 */

#ifndef INC_USEFUL_FUNCTIONS_H_
#define INC_USEFUL_FUNCTIONS_H_
#include <vector>
#include <array>
#include "config.h"
#include <libsc/k60/ov7725.h>
#include <libsc/k60/ov7725_configurator.h>
#include <libsc/led.h>
#include <libsc/lcd_typewriter.h>
#include <libsc/st7735r.h>
#include <libsc/lcd.h>

std::vector<float> linear_regression(std::vector<std::pair<int,int>> m_vector);

double find_slope(std::vector<std::pair<int,int>> m_vector);

std::vector<std::pair<int,int>> harris_corner_detection(const Byte* camBuffer);

std::vector<std::pair<int,int>> susan_corner_detection(const Byte* camBuffer);


#endif /* INC_USEFUL_FUNCTIONS_H_ */
