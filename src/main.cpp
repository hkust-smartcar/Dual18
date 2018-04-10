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
#define pi 3.1415926

#define Master


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

enum RoadType{
	straight,
	right_turn,
	left_turn
};

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

int servo_control(vector<pair<int,int>> midline, vector<pair<int,int>> m_master_vector, vector<pair<int,int>> temp, int &roadtype){
	int servo_degree=1000;
	int x_coord=0;
	double percent=0;
	for(int i=0; i<midline.size(); i++){
		x_coord += midline[i].first;
	}
	if(midline.size()!=0){
		x_coord /= midline.size();
	}
	else{
		x_coord = 40;
	}

	percent = (x_coord-40.0)/80;

	if((temp.size()==0)){
		roadtype = RoadType::right_turn;
		percent *= 1.5;
		servo_degree = servo_degree - percent*710;
	}
	else if((m_master_vector.size()==0)){
		roadtype = RoadType::left_turn;
		percent *= 1.5;
		servo_degree = servo_degree - percent*710;
	}
//	if(percent<0.3){
//		servo_degree = servo_degree - percent*500;
//	}
	else{
		roadtype = RoadType::straight;
		servo_degree = servo_degree - percent*400;

	}
	if(servo_degree < 700){
		servo_degree = 700;
	}else if(servo_degree > 1300){
		servo_degree = 1300;
	}
	return servo_degree;
}


morris_pid my_motor_pid(true);
morris_pid my_servo_pid(false);


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

    DirEncoder dirEncoder0(myConfig::GetEncoderConfig(0));
    DirEncoder dirEncoder1(myConfig::GetEncoderConfig(1));
//    PID servoPID(2500,40000);
    PID motorLPID(0.32,0.0,8, &dirEncoder1);
    PID motorRPID(0.32,0.0,8, &dirEncoder0);
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
	bool control = false;
#ifdef Master
	int right_motor_speed = 180;
	int left_motor_speed = 180;
	bool turn_on_motor = false;

#endif

#ifdef Slave
	int right_motor_speed = 180;
	int left_motor_speed = 180;
#endif
	int changed = 0;

    Joystick js(myConfig::GetJoystickConfig(Joystick::Listener([&changed, & right_motor_speed, &left_motor_speed, &lcd,&right_motor, &left_motor, &start, &led2, &control, &mode, &my_motor_pid, &turn_on_motor](const uint8_t id, const Joystick::State state){
		if(state == Joystick::State::kLeft){
			changed = 1;
			if(mode == 0){
				mode = 0;
			}
			else{
				mode--;
			}

		}
		else if(state == Joystick::State::kRight ){
			changed = 1;
			if(mode == 2){
				mode = 2;
			}
			else{
				mode++;
			}
		}
		else if (state == Joystick::State::kSelect){
			stop++;
//			led2.Switch();
			if(stop%2==1){
				start = true;
				control = true;
				turn_on_motor = true;

			}
			else{
				start = false;
				control = false;
				right_motor.SetPower(0);
				left_motor.SetPower(0);
				turn_on_motor = false;
			}
		}
		else if (state == Joystick::State::kUp){
			if(mode==0){
				right_motor_speed += 5;
				left_motor_speed += 5;
			}
			else if(mode==1){
				my_motor_pid.kp += 0.01;
			}
		}
		else if (state == Joystick::State::kDown){
			if(mode==0){
				right_motor_speed -= 5;
				left_motor_speed -= 5;
			}
			else if(mode==1){
				if(my_motor_pid.kp==0){
					my_motor_pid.kp = 0;
				}
				else{
					my_motor_pid.kp -= 0.01;
				}
			}
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
	#ifdef Master
	const bool is_slave = false;//different MCU, different value
	vector<pair<int,int>> temp;
	vector<pair<int,int>> midline;
	M_Bluetooth m_master_bluetooth;
	vector<pair<int,int>> master_min_xy;
	vector<pair<int,int>> master_max_xy;
	vector<pair<int,int>> slave_min_xy;
	vector<pair<int,int>> slave_max_xy;
	vector<pair<int,int>> midline_min_xy;
	vector<pair<int,int>> midline_max_xy;

	#endif

	#ifdef Slave
	const bool is_slave = true;
	S_Bluetooth m_slave_bluetooth;
	vector<pair<int,int>> slave_min_xy;
	vector<pair<int,int>> slave_max_xy;
	#endif

	bool work = false;
	bool do_not_change_m = false;
	bool do_not_change_s = false;

    vector<pair<int,int>> m_slave_vector;
    vector<pair<int,int>> m_master_vector;
    vector<pair<int,int>> m_vector;
    int pre_x_coord = 40;

    int servo_degree = 1000;

    while(1){
		if(System::Time() != lastTime){

			lastTime = System::Time();
			if(lastTime % 100 == 0){
//				mBT.sendVelocity();
			}

			if(lastTime % 16 == 0){

			}

			if (lastTime % 10 == 0){
//				led0.Switch();
//				led1.Switch();
//				led2.Switch();
//				led3.Switch();
				const Byte* camBuffer = camera.LockBuffer();
                camera.UnlockBuffer();

//                Facing m_facing(&servo, &led0, &led1, &led2, &led3, camBuffer, &right_motor, &left_motor);

                for (int i=0; i<min_xy.size(); i++){
                		min_xy.erase(min_xy.begin());
                }
                for (int i=0; i<max_xy.size(); i++){
                		max_xy.erase(max_xy.begin());
                }
#ifdef Master
                for (int i=0; i<master_min_xy.size(); i++){
                		master_min_xy.erase(master_min_xy.begin());
                }
                for (int i=0; i<master_max_xy.size(); i++){
                		master_max_xy.erase(master_max_xy.begin());
                }
                for (int i=0; i<slave_min_xy.size(); i++){
                		slave_min_xy.erase(slave_min_xy.begin());
                }
                for (int i=0; i<slave_max_xy.size(); i++){
                		slave_max_xy.erase(slave_max_xy.begin());
                }
                for (int i=0; i<midline_min_xy.size(); i++){
                		midline_min_xy.erase(midline_min_xy.begin());
                }
                for (int i=0; i<midline_max_xy.size(); i++){
                		midline_max_xy.erase(midline_max_xy.begin());
                }
#endif

#ifdef Slave
                for (int i=0; i<slave_min_xy.size(); i++){
                		slave_min_xy.erase(slave_min_xy.begin());
                }
                for (int i=0; i<slave_max_xy.size(); i++){
                		slave_max_xy.erase(slave_max_xy.begin());
                }
#endif


// bluetooth send image
#ifdef Slave
                double slave_slope;
				check_right_edge(camBuffer, m_slave_vector);
				m_slave_bluetooth.send_edge(m_slave_vector);
				slave_slope = find_slope(m_slave_vector, slave_min_xy, slave_max_xy);

#endif

#ifdef Master
//				dirEncoder0.Update();
				int left_encoder_velocity = dirEncoder0.GetCount();
//				dirEncoder1.Update();
				int right_encoder_velocity = dirEncoder1.GetCount();
				left_encoder_velocity = -left_encoder_velocity;


				midline.clear();
				temp.clear();
				double master_slope;
				double slave_slope;
				double midline_slope;
				bool is_turn = false;
				int roadtype = RoadType::straight;

				temp = m_master_bluetooth.get_m_edge();


				check_left_edge(camBuffer, m_master_vector);
				master_slope = find_slope(m_master_vector, master_min_xy, master_max_xy);
				slave_slope = find_slope(temp, slave_min_xy, slave_max_xy);


				find_midline(m_master_vector, temp, midline);

				midline_slope = find_slope(midline, midline_min_xy, midline_max_xy);



				if(midline.size()>5){
					servo_degree = servo_control(midline, m_master_vector, temp, roadtype);
				}

				else{
//					servo_degree = 1000;
				}

				servo.SetDegree(servo_degree);

				m_master_bluetooth.reset_m_edge();

//				if((turn_on_motor)&&(roadtype==straight)){
//					left_motor_speed = my_motor_pid.calculate_pid(left_encoder_velocity, 1700);
//					right_motor_speed = my_motor_pid.calculate_pid(right_encoder_velocity, 1700);
//				}
//
//				else if((turn_on_motor)&&(roadtype==right_turn)){
//					left_motor_speed = my_motor_pid.calculate_pid(left_encoder_velocity, 1600);
//					right_motor_speed = my_motor_pid.calculate_pid(right_encoder_velocity, 1500);
//				}
//
//				else if((turn_on_motor)&&(roadtype==left_turn)){
//					left_motor_speed = my_motor_pid.calculate_pid(left_encoder_velocity, 1500);
//					right_motor_speed = my_motor_pid.calculate_pid(right_encoder_velocity, 1600);
//				}

				if(roadtype==left_turn){
					led2.Switch();
				}
				else if(roadtype==right_turn){
					led3.Switch();
				}

				if(turn_on_motor){
					right_motor.SetPower(motorRPID.getPID());
					left_motor.SetPower(motorLPID.getPID());
					motorLPID.setDesiredVelocity(60);
					motorRPID.setDesiredVelocity(60);
				}
				else{
					right_motor.SetPower(0);
					left_motor.SetPower(0);
					my_motor_pid.set_integral(0.0);
				}



#endif




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



				if(mode == 0){
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

#ifdef Slave
					for(int i=0; i<m_slave_vector.size(); i++){
							lcd.SetRegion(Lcd::Rect(m_slave_vector[i].first, m_slave_vector[i].second, 2, 2));
							lcd.FillColor(Lcd::kBlue);
					}
					lcd.SetRegion(Lcd::Rect(0,60,88,15));
					sprintf(c,"Slave");
					writer.WriteBuffer(c,10);
					lcd.SetRegion(Lcd::Rect(0,75,88,15));
					sprintf(c,"Sl:%.2f ", slave_slope);
					writer.WriteBuffer(c,10);
					for(int i=0; i<10; i++){
						c[i] = ' ';
					}
					lcd.SetRegion(Lcd::Rect(0,90,88,15));
					sprintf(c,"Ve:%d ", left_motor_speed);
					writer.WriteBuffer(c,10);
#endif

#ifdef Master
					for(int i=0; i<m_master_vector.size(); i++){
							lcd.SetRegion(Lcd::Rect(m_master_vector[i].first, m_master_vector[i].second, 2, 2));
							lcd.FillColor(Lcd::kRed);
					}
					for(int i=0; i<temp.size(); i++){
							lcd.SetRegion(Lcd::Rect(temp[i].first, temp[i].second, 2, 2));
							lcd.FillColor(Lcd::kGreen);
					}
					lcd.SetRegion(Lcd::Rect(temp[0].first, temp[0].second, 2, 2));
					lcd.FillColor(Lcd::kPurple);

					for(int i=0; i<midline.size(); i++){
							lcd.SetRegion(Lcd::Rect(midline[i].first, midline[i].second, 2, 2));
							lcd.FillColor(Lcd::kBlue);
					}
					lcd.SetRegion(Lcd::Rect(0,60,88,15));
					sprintf(c,"B_Sl:%.2f",midline_slope);
					writer.WriteBuffer(c,10);
					lcd.SetRegion(Lcd::Rect(0,75,88,15));
					sprintf(c,"R_Sl:%.2f ", master_slope);
					writer.WriteBuffer(c,10);
					lcd.SetRegion(Lcd::Rect(0,90,88,15));
					sprintf(c,"G_Sl:%.2f ", slave_slope);
					writer.WriteBuffer(c,10);
					for(int i=0; i<10; i++){
						c[i] = ' ';
					}
					lcd.SetRegion(Lcd::Rect(0,105,88,15));
					sprintf(c,"Sv:%d ", servo_degree);
					writer.WriteBuffer(c,10);
					for(int i=0; i<10; i++){
						c[i] = ' ';
					}
					lcd.SetRegion(Lcd::Rect(0,120,88,15));
					sprintf(c,"R:%d ", right_motor_speed);
					writer.WriteBuffer(c,10);
					for(int i=0; i<10; i++){
						c[i] = ' ';
					}
					lcd.SetRegion(Lcd::Rect(0,135,88,15));
					sprintf(c,"L:%d ", left_motor_speed);
					writer.WriteBuffer(c,10);

#endif
				}

				if(mode == 1){
					char c[10];
					if(changed == 1){
						lcd.Clear();
						changed = 0;
					}
					for(int i=0; i<10; i++){
						c[i] = ' ';
					}
					lcd.SetRegion(Lcd::Rect(0,0,88,15));
					sprintf(c,"m_kp:%.3f ", my_motor_pid.get_kp());
					writer.WriteBuffer(c,10);
					for(int i=0; i<10; i++){
						c[i] = ' ';
					}
					lcd.SetRegion(Lcd::Rect(0,15,88,15));
					sprintf(c,"m_ki:%.3f ", my_motor_pid.get_ki());
					writer.WriteBuffer(c,10);
					for(int i=0; i<10; i++){
						c[i] = ' ';
					}
					lcd.SetRegion(Lcd::Rect(0,30,88,15));
					sprintf(c,"m_kd:%.3f ", my_motor_pid.get_kd());
					writer.WriteBuffer(c,10);
					for(int i=0; i<10; i++){
						c[i] = ' ';
					}
					lcd.SetRegion(Lcd::Rect(0,45,88,15));
					sprintf(c,"s_kp:%.3f ", my_servo_pid.get_kp());
					writer.WriteBuffer(c,10);
					for(int i=0; i<10; i++){
						c[i] = ' ';
					}
					lcd.SetRegion(Lcd::Rect(0,60,88,15));
					sprintf(c,"s_ki:%.3f ", my_servo_pid.get_ki());
					writer.WriteBuffer(c,10);
					for(int i=0; i<10; i++){
						c[i] = ' ';
					}
					lcd.SetRegion(Lcd::Rect(0,75,88,15));
					sprintf(c,"s_kd:%.3f ", my_servo_pid.get_kd());
					writer.WriteBuffer(c,10);
					for(int i=0; i<10; i++){
						c[i] = ' ';
					}
					lcd.SetRegion(Lcd::Rect(0,90,88,15));
					sprintf(c,"R:%d ", right_motor_speed);
					writer.WriteBuffer(c,10);
					for(int i=0; i<10; i++){
						c[i] = ' ';
					}
					lcd.SetRegion(Lcd::Rect(0,105,88,15));
					sprintf(c,"L:%d ", left_motor_speed);
					writer.WriteBuffer(c,10);
					for(int i=0; i<10; i++){
						c[i] = ' ';
					}
					lcd.SetRegion(Lcd::Rect(0,120,88,15));
					sprintf(c,"l_en:%d ", left_encoder_velocity);
					writer.WriteBuffer(c,10);
					for(int i=0; i<10; i++){
						c[i] = ' ';
					}
					lcd.SetRegion(Lcd::Rect(0,135,88,15));
					sprintf(c,"r_en:%d ", right_encoder_velocity);
					writer.WriteBuffer(c,10);
					for(int i=0; i<10; i++){
						c[i] = ' ';
					}

				}

				if(mode == 2){
					if(changed == 1){
						lcd.Clear();
						changed = 0;
					}
				}

				m_vector.clear();
				m_slave_vector.clear();
				m_master_vector.clear();
			}

			if (lastTime % 100 == 0){


			}
		}
    }
    return 0;
}
