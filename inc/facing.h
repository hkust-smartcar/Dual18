/*
 * facing.h
 *
 *  Created on: Feb 28, 2018
 *      Author: morristseng
 */

#ifndef INC_FACING_H_
#define INC_FACING_H_
#include "edge.h"
#include <iostream>
#include "config.h"
#include <libsc/k60/ov7725.h>
#include <libsc/k60/ov7725_configurator.h>
#include <libsc/led.h>
#include <libsc/lcd_typewriter.h>
#include <libsc/st7735r.h>
#include <libsc/lcd.h>
#include "libsc/servo.h"
#include <vector>
using namespace std;

class Facing{
public:
	Facing(FutabaS3010* servo, Led* led0, Led* led1, Led* led2, Led* led3, const Byte* camBuffer, AlternateMotor* right_motor, AlternateMotor* left_motor):
		m_servo(servo),m_led0(led0),m_led1(led1),m_led2(led2),m_led3(led3),m_camBuffer(camBuffer), m_right_motor(right_motor), m_left_motor(left_motor)
	{}
	bool is_facing();
	double cal_slope(vector<pair<int,int>> m_vector);
	void when_on_turn(bool exit_turn);
	void when_on_straight(bool exit);
	bool get_exit();
	bool get_exit_straight();
	bool get_exit_turn();

	bool m_exit_straight  = false;
	bool m_exit_turn = false;



private:
	FutabaS3010* m_servo;
	Led* m_led0;
	Led* m_led1;
	Led* m_led2;
	Led* m_led3;
	const Byte* m_camBuffer;
	double m_slope = 0;
	bool facing = true;
	bool m_exit = false;
	AlternateMotor* m_right_motor;
	AlternateMotor* m_left_motor;




	int counter = 0;
	float lastAngle = 0;
	bool first_time = true;
	int degree = 0;
	int turn_degree = 0;
	bool exit = false;
	bool turn_exit = false;
	bool turn =  false;
	int count_time = 0;


};




#endif /* INC_FACING_H_ */
