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
#define pi 3.1415926

#define Master

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

int stop = 0;

inline bool ret_cam_bit(int x, int y, const Byte* camBuffer) {
	return ((camBuffer[y * 10 + x / 8] >> (7 - (x % 8))) & 1); //return 1 if black
}

int servo_control(vector<pair<int, int>> midline, vector<pair<int, int>> m_master_vector, vector<pair<int, int>> temp, int roadtype) {
	int servo_degree = 1000;
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
		servo_degree = servo_degree - percent * 500;
	} else if (roadtype == RoadType::left_turn) {
		servo_degree = servo_degree - percent * 500;
	} else {
		servo_degree = servo_degree - percent * 300;
	}

	if (servo_degree < 750) {
		servo_degree = 750;
	} else if (servo_degree > 1250) {
		servo_degree = 1250;
	}
	return servo_degree;
}


int main() {
	System::Init();
	Ov7725 camera(myConfig::getCameraConfig(Width, Height));
	Led led0(myConfig::GetLedConfig(0));
	Led led1(myConfig::GetLedConfig(1));
	Led led2(myConfig::GetLedConfig(2));
	Led led3(myConfig::GetLedConfig(3));
	FutabaS3010 servo(myConfig::GetServoConfig());
	AlternateMotor right_motor(myConfig::GetMotorConfig(0));
	AlternateMotor left_motor(myConfig::GetMotorConfig(1));
	St7735r lcd(myConfig::GetLcdConfig());
	LcdTypewriter writer(myConfig::GetWriterConfig(&lcd));
	LcdConsole console(myConfig::GetConsoleConfig(&lcd));
	DirEncoder dirEncoder0(myConfig::GetEncoderConfig(0));
	DirEncoder dirEncoder1(myConfig::GetEncoderConfig(1));

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

	bool changed = false;
	int contrast = 0x64;

	Joystick js(myConfig::GetJoystickConfig(Joystick::Listener([&changed, &right_target_ecoder_count, &left_target_ecoder_count,
											&lcd,&right_motor, &left_motor, &start, &led2, &control, &mode, &turn_on_motor, &contrast]
											 (const uint8_t id, const Joystick::State state) {
			if(state == Joystick::State::kLeft) {
				changed = true;
				if(mode == 0) {
					mode = 0;
				}else {
					mode--;
				}
			}
			else if(state == Joystick::State::kRight ) {
				changed = true;
				if(mode == 2) {
					mode = 2;
				}else {
					mode++;
				}
			}
			else if (state == Joystick::State::kSelect) {
				stop++;
				if(stop%2==1) {
					turn_on_motor = true;
				}
				else {
					stop = 0;
					right_motor.SetPower(0);
					left_motor.SetPower(0);
					turn_on_motor = false;
				}
			}
			else if (state == Joystick::State::kUp) {
				if(mode==0) {
					right_target_ecoder_count += 10;
					left_target_ecoder_count += 10;
				}
				else if(mode ==1){
					contrast +=5;
				}
			}
			else if (state == Joystick::State::kDown) {
				if(mode==0) {
					right_target_ecoder_count -= 10;
					left_target_ecoder_count -= 10;
				}
				else if(mode==1) {
					contrast -=5;
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
	vector<pair<int, int>> temp;
	vector<pair<int, int>> m_master_vector;
	vector<pair<int, int>> midline;
	M_Bluetooth m_master_bluetooth;
#endif

#ifdef Slave
	const bool is_slave = true;
	vector<pair<int, int>> m_slave_vector;
	S_Bluetooth m_slave_bluetooth;
#endif

	vector<pair<int, int>> m_vector;

	int servo_degree = 1000;

	while (1) {
		if (System::Time() != lastTime) {

			lastTime = System::Time();

			if (lastTime % 15 == 0) {
				const Byte* camBuffer = camera.LockBuffer();
				camera.UnlockBuffer();

#ifdef Slave
				// bluetooth send image
				double slave_slope;
				bool right_fail;
				camera.ChangeSecialDigitalEffect(0x00, 120);
				right_fail = check_right_edge(20, 60, camBuffer, m_slave_vector);
				m_slave_bluetooth.send_edge(m_slave_vector);
				m_slave_bluetooth.send_info(right_fail);
				slave_slope = find_slope(m_slave_vector);
#endif

#ifdef Master
				int roadtype = RoadType::straight;
				temp.clear();
				camera.ChangeSecialDigitalEffect(0x00, contrast);

				//encoder
				dirEncoder0.Update();
				int right_encoder_velocity = dirEncoder0.GetCount();
				dirEncoder1.Update();
				int left_encoder_velocity = dirEncoder1.GetCount();
				right_encoder_velocity = -right_encoder_velocity;
				//


				//find midline to control
				bool left_fail = true;
				bool right_fail = true;
				right_fail = m_master_bluetooth.get_fail_on_turn();
				left_fail = check_left_edge(20,60,camBuffer, m_master_vector);
				temp = m_master_bluetooth.get_m_edge();
				find_midline(m_master_vector, temp, midline);
				//

				//check slope
				float master_slope;
				master_slope = find_slope(m_master_vector);

				float slave_slope;
				slave_slope = find_slope(temp);

				float midline_slope;
				midline_slope = find_slope(midline);
				//

				//find roadtype
				if ((midline.size() > 5)) {
					if ((temp.size() <= 5)) {
						roadtype = RoadType::right_turn;
					}
					else if ((m_master_vector.size() <= 5)) {
						roadtype = RoadType::left_turn;
					}
					else {
						roadtype = RoadType::straight;
					}
					servo_degree = servo_control(midline, m_master_vector, temp, roadtype);
				}
				if (roadtype == left_turn) {
					led2.Switch();
				} else if (roadtype == right_turn) {
					led3.Switch();
				}
				//

				//check if out of track
				if((right_fail==true)&&(roadtype==RoadType::right_turn)){
					servo_degree = servo_degree*0.9;
				}
				if((left_fail==true)&&(roadtype==RoadType::left_turn)){
					servo_degree = servo_degree*1.1;
				}
				if((right_fail==true)&&(left_fail==true)){}
				else{
					servo.SetDegree(servo_degree);
				}
				//

				//motor control
				if (turn_on_motor) {
					//for const speed
					if((roadtype!=RoadType::left_turn)&&(roadtype!=RoadType::left_turn)){
						if (right_encoder_velocity < right_target_ecoder_count) {
							r_temp++;
						} else if (right_encoder_velocity >= right_target_ecoder_count) {
							r_temp--;
						}
						if (left_encoder_velocity < left_target_ecoder_count) {
							l_temp++;
						} else if (left_encoder_velocity >= left_target_ecoder_count) {
							l_temp--;
						}
					}
					right_motor.SetPower(r_temp);
					left_motor.SetPower(l_temp);

//					if((left_encoder_velocity<10)||(right_encoder_velocity<10)){
//						right_motor.SetPower(0);
//						left_motor.SetPower(0);
//					}
				}
				else {
					r_temp = right_motor_speed;
					l_temp = left_motor_speed;
					right_motor.SetPower(0);
					left_motor.SetPower(0);
				}
				//





#endif

//facing program start here
				if (facing == true) {
					if (do_not_enter == false) {
						if (park == false) {
							slope = find_slope(m_vector);
							if (slope == 0) {
								park = false;
							} else if (slope > 0.3) {
								inner_turn = true;
								turn = false;
								park = true;
								led0.Switch();
							} else if (slope < 0.18) {
								turn = true;
								park = true;
								right_motor_speed = 150;
								left_motor_speed = 150;
								led1.Switch();
							} else {
								park = true;
							}
						}

						if (park == true) {
							m_slope = find_slope(m_vector);

							if ((ret_cam_bit(30, 55, camBuffer) == 0)
									&& (turn == false)
									&& (inner_turn == false)) {
								servo.SetDegree(1000 + m_slope * 350);
								if (first_time) {
									degree = m_slope * 350;
									first_time = false;
								}
							} else if ((ret_cam_bit(40, 55, camBuffer) == 0)
									&& (turn == true)
									&& (inner_turn == false)) {

								servo.SetDegree(1000 + m_slope * 1000);
								if (first_time) {
									turn_degree = m_slope * 1500;
									first_time = false;
								}
							} else if ((ret_cam_bit(45, 55, camBuffer) == 0)
									&& (turn == false)
									&& (inner_turn == true)) {
								//							led0.Switch();
//								led2.Switch();
								if (m_slope > 0.65) {
									m_slope = 0.65;
								}
								servo.SetDegree(1000 + m_slope * 400);
								if (first_time) {
									degree = m_slope * 400;
									first_time = false;
								}
							}

							else {
								if ((turn == true)) {
									//								led0.Switch();
									//								servo.SetDegree(710);//need to be change
									//								right_motor.SetPower(motor_speed);
									//								left_motor.SetPower(motor_speed);
									double alpha = 0;
									double difference = 0;
									alpha = ((turn_degree / 10.0) * pi) / 180;
									difference = 0.13 * alpha;//0.13 == the width of the car
									//								right_motor.SetPower(right_motor_speed);
									left_motor_speed = right_motor_speed + 50
											+ (200 * difference);
									servo.SetDegree(1000 - turn_degree);//need to be change
								} else if ((turn == false)
										&& (inner_turn == false)) {
									double alpha = 0;
									double difference = 0;
									alpha = ((degree / 10.0) * pi) / 180;
									difference = 0.13 * alpha;//0.13 == the width of the car
									left_motor_speed = right_motor_speed
											+ (200 * difference);
									servo.SetDegree(1000 - degree);
								}

								else if ((turn == false)
										&& (inner_turn == true)) {
									led3.Switch();
									double alpha = 0;
									double difference = 0;
									alpha = ((degree / 10.0) * pi) / 180;
									difference = 0.13 * alpha;//0.13 == the width of the car
									left_motor_speed = right_motor_speed
											+ (200 * difference);
									servo.SetDegree(1000);
								}
								//							right_motor.SetPower(0);
								//							left_motor.SetPower(0);
								first_time = true;
								degree = 0;
								exit = true;
								turn_exit = true;
								inner_turn_exit = true;
								park = false;
								do_not_enter = true;
							}
						}
					}
					bool is_black = false;
					if ((turn == false) && (exit == true)) {
						for (int i = 0; i < 70; i++) {
							if (ret_cam_bit(5 + i, 55, camBuffer) == 1) {
								is_black = true;
							}
						}
						if ((is_black == false)) {
							exit = false;
							servo.SetDegree(1000);
							right_motor.SetPower(0);
							left_motor.SetPower(0);
							do_not_enter = false;
							facing = false;
						}
					}

					bool is_inner_black = false;
					if ((turn == false) && (inner_turn == true)
							&& (inner_turn_exit == true)) {
						for (int i = 0; i < 70; i++) {
							if (ret_cam_bit(5 + i, 52, camBuffer) == 1) {
								is_inner_black = true;
							}
						}
						if ((is_inner_black == false)) {
							inner_turn_exit = false;
							facing = false;
							do_not_enter = false;
							servo.SetDegree(1000);
							right_motor.SetPower(0);
							left_motor.SetPower(0);
						}
					}

					bool is_turn_black = false;
					if ((turn == true) && (turn_exit == true)) {
						has_enter_turn_loop = false;
						if (has_enter_turn_loop == false) {
							for (int j = 0; j < 10; j++) {
								for (int i = 0; i < 70; i++) {
									if (ret_cam_bit(5 + i, 50 + j, camBuffer)
											== 1) {
										//									is_turn_black = true;
										has_enter_turn_loop = true;
									}
								}
							}
						}

						if (has_enter_turn_loop) {
							//						led1.Switch();
							bool no_black = true;
							for (int j = 0; j < 20; j++) {
								for (int i = 0; i < 70; i++) {
									if (ret_cam_bit(5 + i, 40 + j, camBuffer)
											== 1) {
										no_black = false;
										//									led2.Switch();
									}
								}
							}
							if (no_black == false) {//having a bug here do something to adjust it tmr
								//							led1.Switch();
								for (int j = 0; j < 20; j++) {
									for (int i = 0; i < 5; i++) {
										if (ret_cam_bit(74 + i, 40 + j,
												camBuffer) == 1) {
											is_turn_black = true;
											has_enter_turn_loop = true;
										}
									}
								}
							} else {
								is_turn_black = true;
							}
						} else {
							is_turn_black = true;
						}
						if ((is_turn_black == false)) {
							turn_exit = false;
							facing = false;
							do_not_enter = false;
							servo.SetDegree(1000);
							right_motor.SetPower(0);
							left_motor.SetPower(0);
							turn = false;
							has_enter_turn_loop = false;
						}
					}
				}
//facing program ends here


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
					for(int i=0; i<m_slave_vector.size(); i++) {
						lcd.SetRegion(Lcd::Rect(m_slave_vector[i].first, m_slave_vector[i].second, 2, 2));
						lcd.FillColor(Lcd::kBlue);
					}
					lcd.SetRegion(Lcd::Rect(0,60,88,15));
					sprintf(c,"Slave");
					writer.WriteBuffer(c,10);
					lcd.SetRegion(Lcd::Rect(0,75,88,15));
					sprintf(c,"line:%d", m_slave_vector.size());
					writer.WriteBuffer(c,10);
					for(int i=0; i<10; i++) {
						c[i] = ' ';
					}
					lcd.SetRegion(Lcd::Rect(0,90,88,15));
					sprintf(c,"Ve:%d ", left_motor_speed);
					writer.WriteBuffer(c,10);
					for(int i=0; i<10; i++) {
						c[i] = ' ';
					}
					lcd.SetRegion(Lcd::Rect(0,105,88,15));
					sprintf(c,"fail:%d ", right_fail);
					writer.WriteBuffer(c,10);
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
					for (int i = 0; i < 10; i++) {
						c[i] = ' ';
					}
					lcd.SetRegion(Lcd::Rect(0, 120, 40, 15));
					sprintf(c, "R:%d ", right_motor_speed);
					writer.WriteBuffer(c, 10);
					for (int i = 0; i < 10; i++) {
						c[i] = ' ';
					}
					lcd.SetRegion(Lcd::Rect(0, 135, 40, 15));
					sprintf(c, "L:%d ", left_motor_speed);
					writer.WriteBuffer(c, 10);
					lcd.SetRegion(Lcd::Rect(60, 120, 60, 15));
					sprintf(c, "RFail:%d",right_fail);
					writer.WriteBuffer(c, 10);
					lcd.SetRegion(Lcd::Rect(60, 135, 60, 15));
					sprintf(c, "LFail:%d",left_fail);
					writer.WriteBuffer(c, 10);
#endif
				}
#ifdef Master
				if (mode == 1) {
					char c[10];
					if (changed == 1) {
						lcd.Clear();
						changed = 0;
					}
					for (int i = 0; i < 10; i++) {
						c[i] = ' ';
					}
					lcd.SetRegion(Lcd::Rect(0, 45, 88, 15));
					sprintf(c, "cont:%d ", contrast);
					writer.WriteBuffer(c, 10);
					for (int i = 0; i < 10; i++) {
						c[i] = ' ';
					}
					lcd.SetRegion(Lcd::Rect(0, 60, 88, 15));
					sprintf(c, "rsp:%d ", r_temp);
					writer.WriteBuffer(c, 10);
					for (int i = 0; i < 10; i++) {
						c[i] = ' ';
					}
					lcd.SetRegion(Lcd::Rect(0, 75, 88, 15));
					sprintf(c, "lsp:%d ", l_temp);
					writer.WriteBuffer(c, 10);
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
					for (int i = 0; i < 10; i++) {
						c[i] = ' ';
					}
					lcd.SetRegion(Lcd::Rect(0, 120, 88, 15));
					sprintf(c, "r_en:%d ", right_encoder_velocity);
					writer.WriteBuffer(c, 10);
					for (int i = 0; i < 10; i++) {
						c[i] = ' ';
					}
					lcd.SetRegion(Lcd::Rect(0, 135, 88, 15));
					sprintf(c, "l_en:%d ", left_encoder_velocity);
					writer.WriteBuffer(c, 10);
					for (int i = 0; i < 10; i++) {
						c[i] = ' ';
					}
				}

				if (mode == 2) {
					if (changed == 1) {
						lcd.Clear();
						changed = 0;
					}
				}

				m_vector.clear();
				m_master_vector.clear();
				midline.clear();
				temp.clear();
#endif

#ifdef Slave
				m_slave_vector.clear();
#endif

			}
		}
	}
	return 0;
}
