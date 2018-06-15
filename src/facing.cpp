/*
 * facing.cpp
 *
 *  Created on: Feb 28, 2018
 *      Author: morristseng
 */
#include "facing.h"
inline bool ret_cam_bit(int x, int y, const Byte* camBuffer) {
    return ((camBuffer[y * 10 + x / 8] >> (7 - (x % 8))) & 1);//return 1 if black
}

bool Facing::is_facing(){
	return facing;
}

double Facing::cal_slope(vector<pair<int,int>> m_vector){
	bool min_find = false;
	bool max_find = false;
	double slope = 0;
	bool park = false;
	vector<pair<int,int>> min_xy;
	vector<pair<int,int>> max_xy;
	vector<pair<int,int>> mid_xy;

	for(int i=0; i<m_vector.size(); i++){
		if((m_vector[i].first<=78)&&(min_find == false)){
			min_find = true;
			min_xy.push_back(make_pair(m_vector[i].first,m_vector[i].second));
			break;
		}
	}

	for(int i=0; i<m_vector.size(); i++){
		if((m_vector[i].first<=78)){
			if(i==0){
				max_xy.push_back(make_pair(m_vector[0].first, m_vector[0].second));
			}
			else if(m_vector[i].second>=m_vector[i-1].second){
				max_xy.erase(max_xy.begin());
				max_xy.push_back(make_pair(m_vector[i].first, m_vector[i].second));
			}
			max_find = true;
		}
	}

	if(min_find==false){
		min_xy.push_back(make_pair(0,0));
		slope = 0.0;
	}
	if(max_find==false){
		max_xy.push_back(make_pair(0,0));
		slope = 0.0;
	}

	if((min_find==true)&&(max_find==true)){
		if(min_xy[0].second == max_xy[0].second){
			slope = 0.0;
		}
		else{
			slope = ((max_xy[0].second - min_xy[0].second)/(1.0*min_xy[0].first - max_xy[0].first));
		}
	}
	m_slope = slope;
	return slope;
}


void Facing::when_on_straight(bool exit){
	if(exit==false){
		if(ret_cam_bit(35, 55,m_camBuffer)==0){
			m_servo->SetDegree(1000 + m_slope*350);
			if(first_time){
				degree = m_slope*350;
			}
		}
		else{
			m_servo->SetDegree(1000-degree);
			m_exit_straight = true;
		}
	}

	else{
		bool is_black = false;
		for(int i=0; i<70; i++){
			if(ret_cam_bit(5+i, 55,m_camBuffer) == 1){
				is_black = true;
			}
		}
		if((is_black==false)&&(exit==true)){
			exit = false;
			m_servo->SetDegree(1000);
			m_right_motor->SetPower(0);
			m_left_motor->SetPower(0);
		}
	}

}

void Facing::when_on_turn(bool exit_turn){
	if(exit_turn == false){
		if((ret_cam_bit(35, 55,m_camBuffer)==0)&&(turn == true)){
	//		m_led1->Switch();
			m_servo->SetDegree(1000 + m_slope*400);
			if(first_time){
				turn_degree = m_slope*500;
				first_time = false;
			}
		}
		else{
			m_servo->SetDegree(700);//need to be change
			m_exit_turn = true;
		}
	}
	else{
		bool is_turn_black = false;

		for(int j=0;j<10; j++){
			for(int i=0; i<70; i++){
				if(ret_cam_bit(5+i, 50+j,m_camBuffer) == 1){
					is_turn_black = true;
				}
			}
		}
		if((is_turn_black==false)&&(turn_exit==true)){
			turn_exit = false;
			m_servo->SetDegree(1000);
			m_right_motor->SetPower(0);
			m_left_motor->SetPower(0);
			turn = false;
		}
	}
}

bool Facing::get_exit(){
	return m_exit;
}

bool Facing::get_exit_straight(){
	return m_exit_straight;
}

bool Facing::get_exit_turn(){
	return m_exit_turn;
}


