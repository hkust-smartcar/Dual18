/*
 * main.cpp
 *
 * Author: 
 * Copyright (c) 2014-2015 HKUST SmartCar Team
 * Refer to LICENSE for details
 */

#include <cmath>
#include <vector>
#include <cassert>
#include <cstring>
#include <string>
#include <libbase/k60/mcg.h>
#include <libsc/system.h>
#include <libsc/led.h>
#include <libsc/k60/jy_mcu_bt_106.h>
#include "libsc/led.h"
#include "libsc/joystick.h"
#include "libsc/st7735r.h"
#include "libsc/battery_meter.h"
#include "libsc/k60/jy_mcu_bt_106.h"
#include "libbase/k60/pit.h"
#include "libsc/lcd_typewriter.h"
#include "libbase/k60/adc.h"
#include "config.h"
#include "PID.h"
#include "bt.h"
#include <libsc/k60/ov7725.h>
#include <libsc/k60/ov7725_configurator.h>
#include "corner.h"
#include "edge.h"
#include "facing.h"
#include "bluetooth.h"
#include "find_midline.h"
#include "morris_pid.h"
#include "useful_functions.h"
#include "DualCar_UART.h"
#define pi 3.1415926

//#define   Slave
#define   Master


namespace libbase {
namespace k60 {
Mcg::Config Mcg::GetMcgConfig() {
	Mcg::Config config;
	config.external_oscillator_khz = 50000;
	config.core_clock_khz = 150000;
	return config;
}
}
}

using libsc::System;
using namespace libsc;
using namespace libbase::k60;
using libsc::k60::JyMcuBt106;

enum RoadType {
	straight, right_turn, left_turn
};
enum Motor {
	right_motor, left_motor
};


//camera
#define Width 80
#define Height 60
bool camptr[Height][Width];
int mode = 0;
int right_motor_speed = 180;
int left_motor_speed = 180;
int r_temp = right_motor_speed;
int l_temp = left_motor_speed;
int right_target_ecoder_count = 100;
int left_target_ecoder_count = 100;

PID m_straight_pid(1,1);
PID m_turn_pid(2,0);



inline bool ret_cam_bit(int x, int y, const Byte* camBuffer) {
	return ((camBuffer[y * 10 + x / 8] >> (7 - (x % 8))) & 1); //return 1 if black
}

int servo_control(vector<pair<int, int>> midline, vector<pair<int, int>> m_master_vector, vector<pair<int, int>> temp, int roadtype, PID servo_pid) {
	int servo_degree = 1050;
	int x_coord = 0;
	float percent = 0;
	for (int i = 0; i < midline.size(); i++) {
		x_coord += midline[i].first;
	}
	if (midline.size() != 0) {
		x_coord /= midline.size();
	} else {
		x_coord = 40;
	}

	percent = (x_coord - 40.0) / 80;

	if (roadtype == RoadType::right_turn) {
//		servo_degree = servo_degree - m_turn_pid.getPID(0,percent);
//		servo_degree = servo_degree - percent*700;
//		servo_degree = servo_degree + servo_pid.getPID(0,percent);
	} else if (roadtype == RoadType::left_turn) {
//		servo_degree = servo_degree - m_turn_pid.getPID(0,percent);
//		servo_degree = servo_degree - percent*700;
//		servo_degree = servo_degree + servo_pid.getPID(0,percent);
	} else {
//		servo_degree = servo_degree + servo_pid.getPID(0,percent);
//		servo_degree = servo_degree - percent*200;
	}
	servo_degree = servo_degree + servo_pid.getPID(0,percent);

	return servo_degree;
}


int main() {
	System::Init();
	AlternateMotor right_motor(myConfig::GetMotorConfig(0));
	AlternateMotor left_motor(myConfig::GetMotorConfig(1));

	Ov7725 camera(myConfig::getCameraConfig(Width, Height));
	Led led0(myConfig::GetLedConfig(0));
	Led led1(myConfig::GetLedConfig(1));
	Led led2(myConfig::GetLedConfig(2));
	Led led3(myConfig::GetLedConfig(3));
	FutabaS3010 servo(myConfig::GetServoConfig());
	St7735r lcd(myConfig::GetLcdConfig());
	LcdTypewriter writer(myConfig::GetWriterConfig(&lcd));
	LcdConsole console(myConfig::GetConsoleConfig(&lcd));
	DirEncoder dirEncoder0(myConfig::GetEncoderConfig(0));
	DirEncoder dirEncoder1(myConfig::GetEncoderConfig(1));
	PID m_right_motor_pid(0,0,0,&dirEncoder0, false);
	PID m_left_motor_pid(0,0,0,&dirEncoder1, false);
	PID servo_pid(0,0);



	camera.Start();

	led0.SetEnable(1);
	led1.SetEnable(1);
	led2.SetEnable(1);
	led3.SetEnable(1);

	right_motor.SetClockwise(false);
	left_motor.SetClockwise(true);

	bool start = false;
	uint32_t lastTime = 0;

	bool control = false;
	bool turn_on_motor = false;


	int contrast = 0x64;

	//wpid value
	float straight_pid[3][2] = {{0.16, 0.125},
								{0.00, 0.00},
								 {0.5, 0.5}};
	float turn_pid[3][2] = {{0.075, 0.075},
							{0.000, 0.000},
							 {0.055, 0.055}};
	float servo_pd[2] = {1400,2000};

	uint32_t send_ms=0;


	int line = 1;
	bool select = true;
	int select_time = 0;
	bool changed = false;


	Joystick js(myConfig::GetJoystickConfig(Joystick::Listener([&]
											 (const uint8_t id, const Joystick::State state) {

			if(state == Joystick::State::kLeft) {
				changed = true;
				select = false;
				select_time = 0;
				line =1;
				if(mode == 0) {
					mode = 0;
				}else {
					mode--;
				}
			}
			else if(state == Joystick::State::kRight ) {
				changed = true;
				select = false;
				select_time = 0;
				line =1;
				if(mode == 3) {
					mode = 3;
				}else {
					mode++;
				}
			}
			else if (state == Joystick::State::kSelect) {
				if(mode ==1){
					select_time++;
					if(select_time%2==1)
						select = true;
					else
						select = false;
				}
				if(mode ==2){
					select_time++;
					if(select_time%2==1)
						select = true;
					else
						select = false;
				}
				if(mode==3){
					select_time++;
					if(select_time%2==1) {
						turn_on_motor = true;
					}
					else {
						right_motor.SetPower(0);
						left_motor.SetPower(0);
						turn_on_motor = false;
					}
				}
			}
			else if (state == Joystick::State::kUp) {
				if(mode==0) {
				}
				else if(mode==1){
					if(!select){
						if(line == 1)
							line=1;
						else
							line --;
					}
					else{
						switch(line%2){
							case 1:
								servo_pd[0]+= 100;
								break;
							case 0:
								servo_pd[1]+= 100;
								break;
						}
					}
				}
				else if(mode==2) {
					if(!select){
						if(line == 1)
							line=1;
						else
							line --;
					}
					else{
						switch(line%8){
							case 1:
								straight_pid[0][0]+= 0.001;
								break;
							case 2:
								straight_pid[1][0]+= 0.001;
								break;
							case 3:
								straight_pid[2][0]+= 0.001;
								break;
							case 4:
								straight_pid[0][1]+= 0.001;
								break;
							case 5:
								straight_pid[1][1]+= 0.001;
								break;
							case 6:
								straight_pid[2][1]+= 0.001;
								break;
							case 7:
								right_target_ecoder_count += 10;
								break;
							case 0:
								left_target_ecoder_count += 10;
								break;
						}
					}
				}
			}
			else if (state == Joystick::State::kDown) {
				if(mode==0) {
					right_target_ecoder_count -= 10;
					left_target_ecoder_count -= 10;
				}
				else if(mode==1){
					if(!select){
						if(line >= 2)
							line=2;
						else
							line ++;
					}
					else{
						switch(line%2){
							case 1:
								servo_pd[0]-= 100;
								if(servo_pd[0]<0)
									servo_pd[0]=0;
								break;
							case 0:
								servo_pd[1]-= 100;
								if(servo_pd[1]<0)
									servo_pd[1]=0;
								break;
						}
					}
				}
				else if(mode==2) {
					if(!select){
						if(line >= 8)
							line=8;
						else
							line ++;
					}
					else{
						switch(line%8){
							case 1:
								straight_pid[0][0]-= 0.001;
								if(straight_pid[0][0]<0)
									straight_pid[0][0] =0;
								break;
							case 2:
								straight_pid[1][0]-= 0.001;
								if(straight_pid[1][0]<0)
									straight_pid[1][0] =0;
								break;
							case 3:
								straight_pid[2][0]-= 0.001;
								if(straight_pid[2][0]<0)
									straight_pid[2][0] =0;
								break;
							case 4:
								straight_pid[0][1]-= 0.001;
								if(straight_pid[0][1]<0)
									straight_pid[0][1] =0;
								break;
							case 5:
								straight_pid[1][1]-= 0.001;
								if(straight_pid[1][1]<0)
									straight_pid[1][1] =0;
								break;
							case 6:
								straight_pid[2][1]-= 0.001;
								if(straight_pid[2][1]<0)
									straight_pid[2][1] =0;
								break;
							case 7:
								right_target_ecoder_count -= 10;
								if(right_target_ecoder_count<0)
									right_target_ecoder_count = 0;
								break;
							case 0:
								left_target_ecoder_count -= 10;
								if(left_target_ecoder_count<0)
									left_target_ecoder_count = 0;
								break;
						}
					}
				}
			}
	})));

	servo.SetDegree(1000);

	//The variables for facing

	bool facing = false;
	bool first_time = true;
	int degree = 0;
	int turn_degree = 0;
	bool turn_exit = false;
	bool inner_turn_exit = false;
	bool turn = false;
	bool inner_turn = false;
	bool exit = false;
	bool park = false;
	double slope = 0;
	double m_slope = 0;
	bool has_enter_turn_loop = false;
	bool do_not_enter = false;




	//bluetooth communicate
#ifdef Master
	DualCar_UART uart(1, Uart::Config::BaudRate::k115200);
	vector<pair<int, int>> temp;
	vector<pair<int, int>> m_master_vector;
	vector<pair<int, int>> midline;
	M_Bluetooth m_master_bluetooth;
	int servo_degree = 1050;
#endif

#ifdef Slave
	const bool is_slave = true;
	vector<pair<int, int>> m_slave_vector;
	S_Bluetooth m_slave_bluetooth;
#endif

	vector<pair<int, int>> m_vector;


	while (1) {
		if (System::Time() != lastTime) {

			lastTime = System::Time();
#ifdef Master
			uart.RunEveryMS();
#endif
			if (lastTime % 15 == 0) {
				const Byte* camBuffer = camera.LockBuffer();
				camera.UnlockBuffer();

#ifdef Slave
				// bluetooth send image

				double slave_slope;
				bool right_fail;
				camera.ChangeSecialDigitalEffect(0x00, 120);
				right_fail = check_right_edge(20, 60, camBuffer, m_slave_vector);

				send_ms++;
				if(send_ms%5==0){
				send_ms = 0;
				m_slave_bluetooth.send_edge(m_slave_vector);
				m_slave_bluetooth.send_info(right_fail);
				}
				slave_slope = find_slope(m_slave_vector);
				vector<Corner> m_corner;
				if(right_fail!=true)
					m_corner = check_corner(camBuffer, 20, 40, false);
#endif

#ifdef Master
				int roadtype = RoadType::straight;
				temp.clear();
				camera.ChangeSecialDigitalEffect(0x00, 120);


//				//encoder
				if(mode!=3)
					dirEncoder0.Update();
				int right_encoder_velocity = dirEncoder0.GetCount();
				if(mode!=3)
					dirEncoder1.Update();
				int left_encoder_velocity = dirEncoder1.GetCount();
				right_encoder_velocity = -right_encoder_velocity;

				servo_pid.setkP(servo_pd[0]);
				servo_pid.setkD(servo_pd[1]);
//
//
//				//find midline to control
				bool left_fail = true;
				bool right_fail = true;
				right_fail = m_master_bluetooth.get_fail_on_turn();

				left_fail = check_left_edge(20,60,camBuffer, m_master_vector);

				temp = m_master_bluetooth.get_m_edge();
				find_midline(m_master_vector, temp, midline);

//				vector<pair<int,int>> m_corner;
				vector<Corner> m_corner;
				if((left_fail!=true)&&(right_fail!=true))
					m_corner = check_corner(camBuffer, 20, 40, true);


				//

				//check slope
				float master_slope;
				master_slope = find_slope(m_master_vector);
//				uart.Send_float(DualCar_UART::FLOAT::f0, master_slope);

				float slave_slope;
				slave_slope = find_slope(temp);
//				uart.Send_float(DualCar_UART::FLOAT::f1, slave_slope);

				float midline_slope;
				midline_slope = find_slope(midline);
//				uart.Send_float(DualCar_UART::FLOAT::f2, midline_slope);
				//

				//find roadtype
				if ((midline.size() > 5)) {
					if ((temp.size() <= 3)&&(find_slope(midline)>-1)) {
						roadtype = RoadType::right_turn;

					}
					else if ((m_master_vector.size() <= 3)&&(find_slope(midline)<1)) {
						roadtype = RoadType::left_turn;
					}
					else {
						roadtype = RoadType::straight;
					}
					servo_degree = servo_control(midline, m_master_vector, temp, roadtype, servo_pid);
				}

				if (roadtype == left_turn) {
					led2.Switch();
				} else if (roadtype == right_turn) {
					led3.Switch();
				}
				//
				if(roadtype == RoadType::straight){
					m_right_motor_pid.setDesiredVelocity(right_target_ecoder_count);
					m_left_motor_pid.setDesiredVelocity(left_target_ecoder_count);
					m_right_motor_pid.setkP(straight_pid[0][0]);
					m_left_motor_pid.setkP(straight_pid[0][1]);
					m_right_motor_pid.setkI(straight_pid[1][0]);
					m_left_motor_pid.setkI(straight_pid[1][1]);
					m_right_motor_pid.setkD(straight_pid[2][0]);
					m_left_motor_pid.setkD(straight_pid[2][1]);
				}
				else{
					m_right_motor_pid.setDesiredVelocity(75);
					m_left_motor_pid.setDesiredVelocity(75);
					m_right_motor_pid.setkP(straight_pid[0][0]);
					m_left_motor_pid.setkP(straight_pid[0][1]);
					m_right_motor_pid.setkI(straight_pid[1][0]);
					m_left_motor_pid.setkI(straight_pid[1][1]);
					m_right_motor_pid.setkD(straight_pid[2][0]);
					m_left_motor_pid.setkD(straight_pid[2][1]);
				}



				//check if out of track
				if((right_fail==true)&&(left_fail==true)){
					if(servo_degree>1050){
						servo_degree += 50;
					}
					else{
						servo_degree -=50;
					}
				}
				else{
					if((left_fail!=true)&&(right_fail!=true)){
						servo_degree = servo_control(midline, m_master_vector, temp, roadtype, servo_pid);
					}
					else{
						if((right_fail==true)&&(roadtype==RoadType::right_turn)){
							servo_degree = servo_control(midline, m_master_vector, temp, roadtype, servo_pid);
						}
						if((left_fail==true)&&(roadtype==RoadType::left_turn)){

							servo_degree = servo_control(midline, m_master_vector, temp, roadtype, servo_pid);
						}
					}
				}

				if((left_fail==true)&&(right_fail==true)){
					bool nothing = true;
					for(int i=0; i<78; i++){
						for(int j=20; j<60; j++){
							if(ret_cam_bit(i,j, camBuffer)){
								nothing = false;
								break;
							}
						}
						if(nothing==false)
							break;
					}
					if(nothing)
						servo_degree = 1050;
				}



				if (servo_degree <= 820) {
						servo_degree = 820;
					}
				else if (servo_degree >= 1280) {
						servo_degree = 1280;
					}
				servo.SetDegree(servo_degree);



				//motor control
				if (turn_on_motor) {
					r_temp = m_right_motor_pid.getPID();
					l_temp = m_left_motor_pid.getPID();
					if(straight){
						right_motor.SetPower(250);
						left_motor.SetPower(250);
					}
					else{
						right_motor.SetPower(150);
						left_motor.SetPower(150);
					}
				}
				else {
					r_temp = right_motor_speed;
					l_temp = left_motor_speed;
					right_motor.SetPower(0);
					left_motor.SetPower(0);
				}
				//
				uart.Send_float(DualCar_UART::FLOAT::f0, m_left_motor_pid.getcurrentVelocity());
				uart.Send_float(DualCar_UART::FLOAT::f2, m_right_motor_pid.getcurrentVelocity());
				uart.Send_float(DualCar_UART::FLOAT::f4,l_temp);
				uart.Send_float(DualCar_UART::FLOAT::f6, r_temp);
				uart.Send_float(DualCar_UART::FLOAT::f8, roadtype);
				uart.Send_float(DualCar_UART::FLOAT::f10, midline_slope);
				uart.Send_corners(m_corner);
//				if(((right_encoder_velocity>1000)||(left_encoder_velocity>1000)) && (mode=3)){
//					break;
//				}





#endif

				if (mode == 0) {
					if (changed == 1) {
						lcd.Clear();
						changed = 0;
					}
					char c[10];
					for (int i = 0; i < 10; i++) {
						c[i] = ' ';
					}
					lcd.SetRegion(Lcd::Rect(0, 0, Width, Height));
					lcd.FillBits(0x0000, 0xFFFF, camBuffer, Width * Height);

#ifdef Slave
//					for(int i=0; i<m_slave_vector.size(); i++) {
//						lcd.SetRegion(Lcd::Rect(m_slave_vector[i].first, m_slave_vector[i].second, 2, 2));
//						lcd.FillColor(Lcd::kBlue);
//					}
//					for (int i = 0; i < m_corner.size(); i++) {
//						lcd.SetRegion(Lcd::Rect(m_corner[i].get_xcoord(), m_corner[i].get_ycoord(),2, 2));
//						lcd.FillColor(Lcd::kRed);
//					}
//					if(m_corner.size()>0){
//					Corner min_corner = find_min(m_corner);
//					lcd.SetRegion(Lcd::Rect(min_corner.get_xcoord(), min_corner.get_ycoord(),2, 2));
//					lcd.FillColor(Lcd::kPurple);
//					}
//					lcd.SetRegion(Lcd::Rect(0,60,88,15));
//					sprintf(c,"Slave");
//					writer.WriteBuffer(c,10);
//					lcd.SetRegion(Lcd::Rect(0,75,88,15));
//					sprintf(c,"line:%d", m_slave_vector.size());
//					writer.WriteBuffer(c,10);
//					for(int i=0; i<10; i++) {
//						c[i] = ' ';
//					}
//					lcd.SetRegion(Lcd::Rect(0,90,88,15));
//					sprintf(c,"Ve:%d ", left_motor_speed);
//					writer.WriteBuffer(c,10);
//					for(int i=0; i<10; i++) {
//						c[i] = ' ';
//					}
//					lcd.SetRegion(Lcd::Rect(0,105,88,15));
//					sprintf(c,"fail:%d ", right_fail);
//					writer.WriteBuffer(c,10);
#endif

#ifdef Master
					for (int i = 0; i < m_master_vector.size(); i++) {
						lcd.SetRegion(Lcd::Rect(m_master_vector[i].first,m_master_vector[i].second, 2, 2));
						lcd.FillColor(Lcd::kYellow);
					}
					for (int i = 0; i < temp.size(); i++) {
						lcd.SetRegion(Lcd::Rect(temp[i].first, temp[i].second, 2, 2));
						lcd.FillColor(Lcd::kGreen);
					}
					for (int i = 0; i < midline.size(); i++) {
						lcd.SetRegion(Lcd::Rect(midline[i].first, midline[i].second,2, 2));
						lcd.FillColor(Lcd::kBlue);
					}
					for (int i = 0; i < m_corner.size(); i++) {
						lcd.SetRegion(Lcd::Rect(m_corner[i].get_xcoord(), m_corner[i].get_ycoord(),2, 2));
						lcd.FillColor(Lcd::kRed);
					}
					if(m_corner.size()>0){
					Corner min_corner = find_min(m_corner);
					lcd.SetRegion(Lcd::Rect(min_corner.get_xcoord(), min_corner.get_ycoord(),2, 2));
					lcd.FillColor(Lcd::kPurple);
					}

					lcd.SetRegion(Lcd::Rect(0, 60, 88, 15));
					sprintf(c, "RoadT:%d", roadtype);
					writer.WriteBuffer(c, 10);
					lcd.SetRegion(Lcd::Rect(0, 75, 88, 15));
					sprintf(c, "M_Sl:%.2f ", master_slope);
					writer.WriteBuffer(c, 10);
					lcd.SetRegion(Lcd::Rect(0, 90, 88, 15));
					sprintf(c, "S_Sl:%.2f ", slave_slope);
					writer.WriteBuffer(c, 10);
					for (int i = 0; i < 10; i++) {
						c[i] = ' ';
					}
					lcd.SetRegion(Lcd::Rect(0, 105, 88, 15));
					sprintf(c, "Sv:%d ", servo_degree);
					writer.WriteBuffer(c, 10);

					lcd.SetRegion(Lcd::Rect(0, 120, 60, 15));
					sprintf(c, "RFail:%d",right_fail);
					writer.WriteBuffer(c, 10);
					lcd.SetRegion(Lcd::Rect(0, 135, 60, 15));
					sprintf(c, "LFail:%d",left_fail);
					writer.WriteBuffer(c, 10);
#endif
				}
#ifdef Master
				if (mode == 1) {
					char c[15];
					if (changed == 1) {
						lcd.Clear();
						changed = 0;
					}
					for (int i = 0; i < 15; i++) {
						c[i] = ' ';
					}
					lcd.SetRegion(Lcd::Rect(0, 0, 88, 15));
					sprintf(c, "sr_kp:%.3f", servo_pd[0]);
					writer.WriteBuffer(c, 15);

					for (int i = 0; i < 15; i++) {
						c[i] = ' ';
					}
					lcd.SetRegion(Lcd::Rect(0, 15, 88, 15));
					sprintf(c, "sr_kd:%.3f", servo_pd[1]);
					writer.WriteBuffer(c, 15);

					for (int i = 0; i < 15; i++) {
						c[i] = ' ';
					}
					lcd.SetRegion(Lcd::Rect(0, 90, 88, 15));
					sprintf(c, "r_en:%d", right_encoder_velocity);
					writer.WriteBuffer(c, 15);

					for (int i = 0; i < 15; i++) {
						c[i] = ' ';
					}
					lcd.SetRegion(Lcd::Rect(0, 105, 88, 15));
					sprintf(c, "l_en:%d", left_encoder_velocity);
					writer.WriteBuffer(c, 15);
					for (int i = 0; i < 15; i++) {
						c[i] = ' ';
					}
					lcd.SetRegion(Lcd::Rect(0, 120, 88, 15));
					sprintf(c, "line:%d", line);
					writer.WriteBuffer(c, 15);

					for (int i = 0; i < 15; i++) {
						c[i] = ' ';
					}
					if(select==true){
						lcd.SetRegion(Lcd::Rect(0, 135, 120, 15));
						sprintf(c, "select: true");
						writer.WriteBuffer(c, 15);
					}
					else if(select==false){
						lcd.SetRegion(Lcd::Rect(0, 135, 120, 15));
						sprintf(c, "select: false");
						writer.WriteBuffer(c, 15);
					}

				}
				if (mode == 2) {
					if (changed == 1) {
						lcd.Clear();
						changed = 0;
					}
					char c[15];

					for (int i = 0; i < 15; i++) {
						c[i] = ' ';
					}
					lcd.SetRegion(Lcd::Rect(0, 0, 100, 15));
					sprintf(c, "mr_kp:%.3f", m_right_motor_pid.getkP());
					writer.WriteBuffer(c, 15);

					for (int i = 0; i < 15; i++) {
						c[i] = ' ';
					}
					lcd.SetRegion(Lcd::Rect(0, 15, 100, 15));
					sprintf(c, "mr_ki:%.3f", m_right_motor_pid.getkI());
					writer.WriteBuffer(c, 15);

					for (int i = 0; i < 15; i++) {
						c[i] = ' ';
					}
					lcd.SetRegion(Lcd::Rect(0, 30, 100, 15));
					sprintf(c, "mr_kd:%.3f", m_right_motor_pid.getkD());
					writer.WriteBuffer(c, 15);

					for (int i = 0; i < 15; i++) {
						c[i] = ' ';
					}
					lcd.SetRegion(Lcd::Rect(0, 45, 100, 15));
					sprintf(c, "ml_kp:%.3f", m_left_motor_pid.getkP());
					writer.WriteBuffer(c, 15);

					for (int i = 0; i < 15; i++) {
						c[i] = ' ';
					}
					lcd.SetRegion(Lcd::Rect(0, 60, 100, 15));
					sprintf(c, "ml_ki:%.3f", m_left_motor_pid.getkI());
					writer.WriteBuffer(c, 15);

					for (int i = 0; i < 15; i++) {
						c[i] = ' ';
					}
					lcd.SetRegion(Lcd::Rect(0, 75, 100, 15));
					sprintf(c, "ml_kd:%.3f", m_left_motor_pid.getkD());
					writer.WriteBuffer(c, 15);

					for (int i = 0; i < 10; i++) {
							c[i] = ' ';
						}
					lcd.SetRegion(Lcd::Rect(0, 90, 88, 15));
					sprintf(c, "rtec:%d ", right_target_ecoder_count);
					writer.WriteBuffer(c, 10);

					for (int i = 0; i < 10; i++) {
						c[i] = ' ';
					}
					lcd.SetRegion(Lcd::Rect(0, 105, 88, 15));
					sprintf(c, "ltec:%d ", left_target_ecoder_count);
					writer.WriteBuffer(c, 10);

					for (int i = 0; i < 15; i++) {
						c[i] = ' ';
					}
					lcd.SetRegion(Lcd::Rect(0, 120, 100, 15));
					sprintf(c, "line:%d", line);
					writer.WriteBuffer(c, 15);

					for (int i = 0; i < 15; i++) {
						c[i] = ' ';
					}
					if(select==true){
						lcd.SetRegion(Lcd::Rect(0, 135, 120, 15));
						sprintf(c, "select: true");
						writer.WriteBuffer(c, 15);
					}
					else if(select==false){
						lcd.SetRegion(Lcd::Rect(0, 135, 120, 15));
						sprintf(c, "select: false");
						writer.WriteBuffer(c, 15);
					}
				}

				if (mode == 3) {
					if (changed == 1) {
						lcd.Clear();
						changed = 0;
					}
				}
				m_master_vector.clear();
				midline.clear();
				temp.clear();
				m_corner.clear();
#endif
				m_vector.clear();

#ifdef Slave
				m_slave_vector.clear();
				m_corner.clear();
#endif

			}
		}
	}
	return 0;
}
