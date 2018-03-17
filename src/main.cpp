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

#define pi 3.1415926

namespace libbase
{
    namespace k60
    {
        Mcg::Config Mcg::GetMcgConfig()
        {
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


char *str = "";

//magnetic
float inv_sqrt(float x){
	float xhalf = 0.5f*x;
	int i = *(int*)&x;
	i = 0x5f3759df - (i >> 1);
	x = *(float*)&i;
	x = x*(1.5f - xhalf*x*x);
	x = x*(1.5f - xhalf*x*x);
	return x;
}

float _sqrt(float x){
	return x*inv_sqrt(x);
}

//camera
#define Width 80
#define Height 60
bool camptr[Height][Width];


int stop = 0;
int mode = 0;

inline bool ret_cam_bit(int x, int y, const Byte* camBuffer) {
    return ((camBuffer[y * 10 + x / 8] >> (7 - (x % 8))) & 1);//return 1 if black
}

double find_slope(vector<pair<int,int>> m_vector, vector<pair<int,int>> &min_xy, vector<pair<int,int>> &max_xy){
	bool min_find = false;
	bool max_find = false;
	double slope;
	for(int i=(m_vector.size()/2); i<m_vector.size(); i++){

		if((m_vector[i].first<=78)&&(min_find == false)){
			min_xy.push_back(make_pair(m_vector[i].first, m_vector[i].second));
			min_find = true;
		}
		if((m_vector[i].first<=78)){
			if(i==(m_vector.size()/2)){
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
	}
	if(max_find==false){
		max_xy.push_back(make_pair(0,0));
	}
	if((min_find==true)&&(max_find==true)){
		if(min_xy[0].second == max_xy[0].second){
			slope = 0.0;
		}
		else{
			slope = ((max_xy[0].second - min_xy[0].second)/(1.0*min_xy[0].first - max_xy[0].first));
		}
	}
	return slope;
}


//main
int main() {
    System::Init();
    Ov7725 camera(myConfig::getCameraConfig(Width,Height));
    camera.Start();

    Led led0(myConfig::GetLedConfig(0));
    Led led1(myConfig::GetLedConfig(1));
    Led led2(myConfig::GetLedConfig(2));
    Led led3(myConfig::GetLedConfig(3));

    led0.SetEnable(1);
    led1.SetEnable(1);
    led2.SetEnable(1);
    led3.SetEnable(1);

	Adc mag0(myConfig::GetAdcConfig(0));
	Adc mag1(myConfig::GetAdcConfig(1));
	Adc mag2(myConfig::GetAdcConfig(2));
	Adc mag3(myConfig::GetAdcConfig(3));

	mag0.StartConvert();
	mag1.StartConvert();
	mag2.StartConvert();
	mag3.StartConvert();

	FutabaS3010 servo(myConfig::GetServoConfig());
	AlternateMotor right_motor(myConfig::GetMotorConfig(0));
	AlternateMotor left_motor(myConfig::GetMotorConfig(1));
	right_motor.SetClockwise(false);
	left_motor.SetClockwise(true);
    St7735r lcd(myConfig::GetLcdConfig());
    LcdTypewriter writer(myConfig::GetWriterConfig(&lcd));
    LcdConsole console(myConfig::GetConsoleConfig(&lcd));
//    lcd.SetRegion(Lcd::Rect(0,0,128,160));
//    lcd.FillColor(lcd.kWhite);

    DirEncoder dirEncoder(myConfig::GetEncoderConfig());
    PID servoPID(2500,40000);
    PID motorLPID(0.3,0.0,0.0, &dirEncoder);
    PID motorRPID(0.6,0.0,0.0, &dirEncoder);
//    bt mBT(&servoPID, &motorLPID, &motorRPID);
    typedef enum {
		normal = 0,
		nearLoop,
		straight,
		turning,
		trigger,
		inside
	}carState;
	carState state = normal;
	bool start = false;
    uint32_t lastTime = 0;
    uint32_t greenTime = 0;
	bool dir = 0;
	uint8_t left_mag, right_mag, mid_l_mag, mid_r_mag;
	float angle = 0;
	float left_x, right_x;
	const float left_k = 767.2497;
	const float right_k = 854.7614;
	const float h = 6.2;//magnetic sensor height
	float magRatio, xRatio;
	bool left = 0;
	uint8_t count = 0;
	bool waitTrigger = 1;
	int motor_speed = 200;
	int right_motor_speed = 200;
	int left_motor_speed = 200;
	int changed = 0;

    Joystick js(myConfig::GetJoystickConfig(Joystick::Listener([&changed, & right_motor_speed, &left_motor_speed, &lcd,&right_motor, &left_motor, &start, &led2](const uint8_t id, const Joystick::State state){
		if(state == Joystick::State::kLeft){//magnetic mode
//			lcd.Clear();
			changed = 1;
			mode = 0;//magnetic mode
		}
		else if(state == Joystick::State::kUp){//camera mode
//			lcd.Clear();
			changed = 1;
			mode = 1;
		}
		else if (state == Joystick::State::kSelect){
			stop++;
			if(stop%2==1){
				start = true;
				right_motor.SetPower(right_motor_speed);
				left_motor.SetPower(left_motor_speed);
			}
			else{
				start = false;
				right_motor.SetPower(0);
				left_motor.SetPower(0);
			}
		}
		else if (state == Joystick::State::kUp){
			right_motor_speed += 5;
			left_motor_speed += 5;
		}
		else if (state == Joystick::State::kDown){
			right_motor_speed -= 5;
			left_motor_speed -= 5;
		}

    })));
	vector<pair<int,int>> min_xy;
	vector<pair<int,int>> max_xy;
	vector<pair<int,int>> mid_xy;

	servo.SetDegree(1000);

	//The variables for facing
	bool facing = false;
	bool first_time = true;
	int degree = 0;
	int turn_degree = 0;
	bool turn_exit = false;
	bool inner_turn_exit = false;
	bool turn =  false;
	bool inner_turn = false;
	bool exit = false;
	bool park = false;
	double slope = 0;
	double m_slope = 0;
	bool has_enter_turn_loop = false;
	bool do_not_enter = false;

	//bluetooth communicate
	const bool is_slave = true;//different MCU, different value
	vector<pair<int,int>> temp;
	bool work = false;
	bool do_not_change_m = false;
	bool do_not_change_s = false;
//	M_Bluetooth m_master_bluetooth;
	S_Bluetooth m_slave_bluetooth;
    vector<pair<int,int>> m_slave_vector;
    vector<pair<int,int>> m_master_vector;
    vector<pair<int,int>> m_vector;

    while(1){
		if(System::Time() != lastTime){

			lastTime = System::Time();
			if(lastTime % 100 == 0){
//				mBT.sendVelocity();
			}

			if (lastTime % 10 == 0){
				const Byte* camBuffer = camera.LockBuffer();
                camera.UnlockBuffer();

//                Facing m_facing(&servo, &led0, &led1, &led2, &led3, camBuffer, &right_motor, &left_motor);

                for (int i=0; i<min_xy.size(); i++){
                		min_xy.erase(min_xy.begin());
                }
                for (int i=0; i<max_xy.size(); i++){
                		max_xy.erase(max_xy.begin());
                }

				left_mag = 0;
				right_mag = 0;
				mid_l_mag = 0;
				mid_r_mag = 0;

//				m_vector = check_corner(camBuffer);
//				m_vector = check_edge(camBuffer);

//				if (left_k*h/left_mag < h*h){
//					left_x = 0;
//				}
//				else{
//					left_x = _sqrt(left_k*h/left_mag-h*h);
//				}
//				if (right_k*h/right_mag < h*h){
//					right_x = 0;
//				}
//				else{
//					right_x = _sqrt(right_k*h/right_mag-h*h);
//				}
//
//				xRatio = (float)left_x/(right_x+left_x);
//				angle = servoPID.getPID(0.5,xRatio);
//				angle += 900;
//				if (angle > 1800) {
//					angle = 1800;
//				}
//				else if (angle < 0){
//					angle = 0;
//				}
//				//trigger
//				if (waitTrigger && left_x+right_x <= 20 && (left_mag > 100 || right_mag > 100)){
//					count++;
//					waitTrigger = 0;
//					if (count % 3 == 1){
//						if (left_mag > 100){
//							left = 1;
//						}
//						else{
//							left = 0;
//						}
//						state = nearLoop;
//					}
//					else if (count % 3 == 2){
//						state = turning;
//					}
//					else{
//						state = normal;
//					}
////					lcd.SetRegion(Lcd::Rect(0,0,100,100));
////					lcd.FillColor(0xFFF0);
//				}
//				if (!waitTrigger && left_mag <= 80 && right_mag <= 80){
//					waitTrigger = 1;
//				}
//
//				if (state == turning){
//					if (left && right_mag <= 40){
//						state = inside;
//					}
//					else if (!left && left_mag <= 40){
//						state = inside;
//					}
//					if (left){
////						servo.SetDegree(1800);
//					}
//					else{
////						servo.SetDegree(0);
//					}
//				}
//				if (state == normal || state == inside){
////					servo.SetDegree(angle);
//				}
//				if (state == nearLoop){
////					servo.SetDegree(angle);
//				}



// bluetooth send image

				if(is_slave){
					check_right_edge(camBuffer, m_slave_vector);
					m_slave_bluetooth.send_edge(m_slave_vector);
				}

//				if(!is_slave){//master
//
//					std::vector<std::pair<int,int>> temp = m_master_bluetooth.get_m_edge();
//					check_left_edge(camBuffer, m_master_vector);
//
//					for(int i=0; i<temp.size(); i++){
//						m_master_vector.emplace_back(temp[i]);
//					}
//					m_master_bluetooth.reset_m_edge();
//					temp.clear();
//				}





//facing program start here
				if(facing == true){
					if(do_not_enter == false){
						if(park==false){
							slope = find_slope(m_vector,min_xy, max_xy);
							if(slope==0){
								park = false;
							}
							else if(slope>0.3){
								inner_turn = true;
								turn = false;
								park = true;
								led0.Switch();
							}
							else if(slope < 0.18){
								turn = true;
								park = true;
								right_motor_speed = 150;
								left_motor_speed = 150;
								led1.Switch();
							}
							else{
								park = true;
							}
						}



						if(park == true){
							m_slope = find_slope(m_vector,min_xy, max_xy);

							if((ret_cam_bit(30, 55,camBuffer)==0)&&(turn == false)&&(inner_turn == false)){
								servo.SetDegree(1000+m_slope*350);
								if(first_time){
									degree = m_slope*350;
									first_time = false;
								}
							}
							else if((ret_cam_bit(40, 55,camBuffer)==0)&&(turn == true)&&(inner_turn == false)){

								servo.SetDegree(1000+m_slope*1000);
								if(first_time){
									turn_degree = m_slope*1500;
									first_time = false;
								}
							}
							else if((ret_cam_bit(45, 55,camBuffer)==0)&&(turn == false)&&(inner_turn == true)){
	//							led0.Switch();
								led2.Switch();
								if(m_slope>0.65){
									m_slope = 0.65;
								}
								servo.SetDegree(1000+m_slope*400);
								if(first_time){
									degree = m_slope*400;
									first_time = false;
								}
							}

							else{
								if((turn == true)){
	//								led0.Switch();
	//								servo.SetDegree(710);//need to be change
	//								right_motor.SetPower(motor_speed);
	//								left_motor.SetPower(motor_speed);
									double alpha = 0;
									double difference = 0;
									alpha = ((turn_degree/10.0)*pi)/180;
									difference = 0.13*alpha;//0.13 == the width of the car
	//								right_motor.SetPower(right_motor_speed);
									left_motor_speed = right_motor_speed+50+(200*difference);
									servo.SetDegree(1000 - turn_degree);//need to be change
								}
								else if ((turn==false)&&(inner_turn == false)){
									double alpha = 0;
									double difference = 0;
									alpha = ((degree/10.0)*pi)/180;
									difference = 0.13*alpha;//0.13 == the width of the car
									left_motor_speed = right_motor_speed+(200*difference);
									servo.SetDegree(1000-degree);
								}

								else if ((turn==false)&&(inner_turn == true)){
									led3.Switch();
									double alpha = 0;
									double difference = 0;
									alpha = ((degree/10.0)*pi)/180;
									difference = 0.13*alpha;//0.13 == the width of the car
									left_motor_speed = right_motor_speed+(200*difference);
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
					if((turn==false)&&(exit==true)){
						for(int i=0; i<70; i++){
							if(ret_cam_bit(5+i, 55,camBuffer) == 1){
								is_black = true;
							}
						}
						if((is_black==false)){
							exit = false;
							servo.SetDegree(1000);
							right_motor.SetPower(0);
							left_motor.SetPower(0);
							do_not_enter = false;
							facing = false;
						}
					}

					bool is_inner_black = false;
					if((turn==false)&&(inner_turn==true)&&(inner_turn_exit == true)){
						for(int i=0; i<70; i++){
							if(ret_cam_bit(5+i, 52,camBuffer) == 1){
								is_inner_black = true;
							}
						}
						if((is_inner_black==false)){
							inner_turn_exit = false;
							facing = false;
							do_not_enter = false;
							servo.SetDegree(1000);
							right_motor.SetPower(0);
							left_motor.SetPower(0);
						}
					}


					bool is_turn_black = false;
					if((turn==true)&&(turn_exit==true)){
						has_enter_turn_loop = false;
						if(has_enter_turn_loop == false){
							for(int j=0;j<10; j++){
								for(int i=0; i<70; i++){
									if(ret_cam_bit(5+i, 50+j,camBuffer) == 1){
	//									is_turn_black = true;
										has_enter_turn_loop = true;
									}
								}
							}
						}

						if(has_enter_turn_loop){
	//						led1.Switch();
							bool no_black = true;
							for(int j=0;j<20; j++){
								for(int i=0; i<70; i++){
									if(ret_cam_bit(5+i, 40+j,camBuffer) == 1){
										no_black = false;
	//									led2.Switch();
									}
								}
							}
							if(no_black == false){//having a bug here do something to adjust it tmr
	//							led1.Switch();
								for(int j=0;j<20; j++){
									for(int i=0; i<5; i++){
										if(ret_cam_bit(74+i, 40+j,camBuffer) == 1){
											is_turn_black = true;
											has_enter_turn_loop = true;
										}
									}
								}
							}
							else{
								is_turn_black = true;
							}
						}
						else{
							is_turn_black = true;
						}
						if((is_turn_black==false)){
	//						led3.Switch();
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



				if(mode == 1){
					if(changed == 1){
						lcd.Clear();
						changed = 0;
					}
					char c[10];
					for(int i=0; i<10; i++){
						c[i] = ' ';
					}
	                lcd.SetRegion(Lcd::Rect(0, 0, Width, Height));
	                lcd.FillBits(0x0000, 0xFFFF, camBuffer, Width * Height);
//	                for(int i=0; i<m_vector.size(); i++){
//	                		lcd.SetRegion(Lcd::Rect(m_vector[i].first, m_vector[i].second, 3, 3));
//	                		lcd.FillColor(Lcd::kRed);
//	                }
	                if(is_slave){
//	                	led0.Switch();
						for(int i=0; i<m_slave_vector.size(); i++){
								lcd.SetRegion(Lcd::Rect(m_slave_vector[i].first, m_slave_vector[i].second, 2, 2));
								lcd.FillColor(Lcd::kBlue);
						}
	                }
	                else{
						for(int i=0; i<m_master_vector.size(); i++){
								lcd.SetRegion(Lcd::Rect(m_master_vector[i].first, m_master_vector[i].second, 2, 2));
								lcd.FillColor(Lcd::kRed);
						}
	                }
//					lcd.SetRegion(Lcd::Rect(0,60,88,15));
//					sprintf(c,"U:%d %d", min_xy[0].first, min_xy[0].second);//min_find
//					writer.WriteBuffer(c,10);
//					lcd.SetRegion(Lcd::Rect(0,75,88,15));
//					sprintf(c,"D:%d %d", max_xy[0].first, max_xy[0].second);//max_find
//					writer.WriteBuffer(c,10);
//					lcd.SetRegion(Lcd::Rect(0,90,88,15));
//					sprintf(c,"M:%.3f", m_slope);//slope
//					writer.WriteBuffer(c,10);
					if(is_slave){
						lcd.SetRegion(Lcd::Rect(0,60,88,15));
						sprintf(c,"Slave");
						writer.WriteBuffer(c,10);
					}
					else{
						lcd.SetRegion(Lcd::Rect(0,60,88,15));
						sprintf(c,"Master");
						writer.WriteBuffer(c,10);
					}
				}
				m_vector.clear();
				m_slave_vector.clear();
				m_master_vector.clear();
			}

			if (lastTime % 100 == 0){
	//				if(start){
	////					dirEncoder.Update();
	////					motor.SetPower(motorLPID.getPID(0 - dirEncoder.GetCount()));
	//				}
	//				else{
				if(mode ==0){
					if(changed == 1){
						lcd.Clear();
						changed = 0;
					}
					char c[10];
					for(int i=0; i<10; i++){
						c[i] = ' ';
					}
					lcd.SetRegion(Lcd::Rect(0,0,88,15));
					sprintf(c,"L: %d ",left_mag);
					writer.WriteBuffer(c,10);
					lcd.SetRegion(Lcd::Rect(0,15,88,15));
					sprintf(c,"R: %d ",right_mag);
					writer.WriteBuffer(c,10);
					lcd.SetRegion(Lcd::Rect(0,30,88,15));
					sprintf(c,"ML: %d ",mid_l_mag);
					writer.WriteBuffer(c,10);
					lcd.SetRegion(Lcd::Rect(0,45,88,15));
					sprintf(c,"MR: %d ",mid_r_mag);
					writer.WriteBuffer(c,10);
					lcd.SetRegion(Lcd::Rect(0,60,88,15));
					sprintf(c,"MS: %d ", motor_speed);
					writer.WriteBuffer(c,10);
				}

			}
		}
    }
    return 0;
}

