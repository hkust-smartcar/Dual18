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
#include "DualCar_UART.h"
#include "libsc/passive_buzzer.h"
#include "libsc/battery_meter.h"
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
int max(int a, int b){return a>b?a:b;}
int min(int a, int b){return a<b?a:b;}

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
	Adc mag4(myConfig::GetAdcConfig(4));
	Adc mag5(myConfig::GetAdcConfig(5));

	mag0.StartConvert();
	mag1.StartConvert();
	mag2.StartConvert();
	mag3.StartConvert();
	mag4.StartConvert();
	mag5.StartConvert();

	BatteryMeter batteryMeter(myConfig::GetBatteryMeterConfig());
	float batteryVoltage = batteryMeter.GetVoltage();
	PassiveBuzzer::Config config;
	PassiveBuzzer buzz(config);
	buzz.SetNote(523);
	buzz.SetBeep(batteryVoltage<7);

	FutabaS3010 servo(myConfig::GetServoConfig());
	AlternateMotor right_motor(myConfig::GetMotorConfig(0));
	AlternateMotor left_motor(myConfig::GetMotorConfig(1));
    St7735r lcd(myConfig::GetLcdConfig());
    LcdTypewriter writer(myConfig::GetWriterConfig(&lcd));
    LcdConsole console(myConfig::GetConsoleConfig(&lcd));
//    lcd.SetRegion(Lcd::Rect(0,0,128,160));
//    lcd.FillColor(lcd.kWhite);

    DirEncoder REncoder(myConfig::GetEncoderConfig(0));
    DirEncoder LEncoder(myConfig::GetEncoderConfig(1));
    PID servoPIDStraight(2200,0.03);
    PID servoPIDCurve(5500,1);//4825,0.5
    PID servoPIDAlignCurve(-10,0);
    PID left_motorPID(14,1,0, &LEncoder,false);
    PID right_motorPID(14,1,0, &REncoder,true);
//    bt mBT(&servoPID, &left_motorPID, &right_motorPID);
	typedef enum {
		normal = 0,
		nearLoop,
		straight1,
		straight2,
		turning,
		inLoop,
		junction,
		outLoop,
		leave,
		align,
		side,
		back
	}carState;
	carState state = normal;
    uint32_t greenTime = 0;
	bool dir = 0;
	uint8_t left_mag, right_mag, mid_l_mag, mid_r_mag;
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

	float testLinear = 0;
	bool start = false, cali = false, noCal = true, approaching = false;
	bool onlyNormal = true, tuningPID = false;
	bool leftLoop = false;
	uint16_t filterSum0 = 0, filterSum1 = 0, filterSum2 = 0, filterSum3 = 0, filterSum4 = 0, filterSum5 = 0;
	uint8_t filterCounter = 0;
	int32_t loopSum = 0;
	uint16_t loopCounter = 0;
	uint32_t lastTime = 0, stateTime = 0, approachTime = 0;

	const uint16_t middleServo = 850, leftServo = 1150, rightServo = 550;
	float angle = middleServo;
	float lastServo = 0;

	volatile uint8_t cycleTime = 0;
    const uint8_t cycle = 10;
    float loopSpeed = 4*cycle, highSpeed = 6*cycle, alignSpeed = 6*cycle;
    float speed = highSpeed;
	float frontLinear, midLinear, diffLinear;
	float raw_frontLinear, raw_midLinear;
	float multiplier = 0.0, top_multiplier = 0.0;
	float front_left = 1, front_right = 1, mid_left = 1, mid_right = 1, back_left = 1, back_right = 1, top_left = 1, top_right = 1;
	uint8_t raw_front_left, raw_front_right, raw_mid_left, raw_mid_right, raw_top_left, raw_top_right;
	volatile float encoderLval, encoderRval;
	int32_t powerL, powerR;
	float pCurve, dCurve, pStraight, dStraight, pleft_motor, ileft_motor, dleft_motor, pright_motor, iright_motor, dright_motor;
	float setAngle = 0;

	uint8_t oneLineMax = 0, oneLineMin = 250, equalMin = 250, equalMax = 0, topMax = 0;
	uint8_t maxLeft = 0,minLeft = 100, maxRight = 0,minRight = 100;
	uint8_t count1, count4;
	//\DualCar_UART uart0; // << BT related
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
		else if(state == Joystick::State::kRight){
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

			//uart0.RunEveryMS(); // << BT related
			filterCounter++;
			filterSum0 += mag0.GetResult();
			filterSum1 += mag1.GetResult();
			filterSum2 += mag2.GetResult();
			filterSum3 += mag3.GetResult();
			filterSum4 += mag4.GetResult();
			filterSum5 += mag5.GetResult();
			if (lastTime % cycle == 0 && filterCounter != 0){
				const Byte* camBuffer = camera.LockBuffer();
                camera.UnlockBuffer();
                raw_mid_left = round(1.0*filterSum0/filterCounter);
                raw_mid_right = round(1.0*filterSum1/filterCounter);
                raw_front_left = round(1.0*filterSum2/filterCounter);
                raw_front_right = round(1.0*filterSum3/filterCounter);
                raw_top_left = round(1.0*filterSum4/filterCounter);
                raw_top_right = round(1.0*filterSum5/filterCounter);
                if (multiplier != 0){
                	mid_left = raw_mid_left*multiplier;
                	mid_right = raw_mid_right*multiplier;
                	front_left = raw_front_left*multiplier;
                	front_right = raw_front_right*multiplier;
                	top_left = raw_top_left*top_multiplier;
                	top_right = raw_top_right*top_multiplier;
                }
                filterSum0 = 0;
                filterSum1 = 0;
                filterSum2 = 0;
                filterSum3 = 0;
                filterSum4 = 0;
                filterSum5 = 0;
                filterCounter = 0;
                if (cali){
                	oneLineMin  = min(raw_front_left, min(raw_front_right, min(raw_mid_left, min(raw_mid_right, oneLineMin))));
                	oneLineMax  = max(raw_front_left, max(raw_front_right, max(raw_mid_left, max(raw_mid_right, oneLineMax))));
                	topMax = max(raw_top_left, max(raw_top_right, topMax));
                	if (raw_front_left == raw_front_right){
                		if (equalMin > 100 || (equalMin>raw_front_left && equalMin-raw_front_left < 5)){
                			equalMin = raw_front_left;

                		}
                		if (equalMax < 10 || (equalMax<raw_front_left && raw_front_left-equalMax < 5)){
                			equalMax = raw_front_left;
                		}
                	}
                }
                if (start && max(max(max(raw_front_left,raw_front_right),raw_mid_left),raw_mid_right) < oneLineMin*2){
                	start = false;
                	left_motorPID.setDesiredVelocity(0);
                	right_motorPID.setDesiredVelocity(0);
                }
                //for alignment
                if (state == normal && approaching){
                	state = leave;
                	speed = alignSpeed;
                }
                else if (state == leave && raw_front_right < equalMin*1.2 && raw_front_left < equalMin*0.5){
                	state = align;
                }
                else if (state == align && (raw_front_right-raw_mid_right)<10 && abs((int)angle-middleServo) < 50){
                	state = side;
                	approachTime = System::Time();
                }
                else if (state == side && !approaching){
                	state = back;
                	lastServo = -300;
                }
                else if (state == back && raw_front_left>equalMin){
                	state = normal;
                	speed = 0;
                //	speed = highSpeed;
                }
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
//				REncoder.Update();
				int left_encoder_velocity = REncoder.GetCount();
//				LEncoder.Update();
				int right_encoder_velocity = LEncoder.GetCount();
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


				if (cali){
					angle = setAngle;
				}else if (state == normal){
					if (raw_front_left < oneLineMin*2.5 || raw_front_right < oneLineMin*2.5){
						angle = lastServo*1.05;//compare with which side and angle sign
					}
					else{
						angle = servoPIDCurve.getPID(0.0,frontLinear);
//						if(IsCurve(frontLinear, midLinear)){
//							angle = servoPIDCurve.getPID(0.0,frontLinear);
//							buzz.SetBeep(true);
//						}else{
//							angle = servoPIDStraight.getPID(0.0,frontLinear);
//							buzz.SetBeep(false);
//						}
					}
					lastServo = angle;
				}else if (state == align){
					angle = servoPIDAlignCurve.getPID(equalMin, raw_front_right);
//					angle = servoPIDCurve.getPID(0.08,frontLinear);
					if (raw_front_left>oneLineMin*3 && angle<0){
						angle = -angle;
					}
				}
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

				angle += middleServo;
				angle = max(rightServo,min(leftServo,angle));
				servo.SetDegree(angle);

				if (angle>middleServo){
					float angleRatio = 0.0013*(angle-middleServo);
					float differential = angleRatio/(2-angleRatio);
					left_motorPID.setDesiredVelocity(speed*(1-differential));
					right_motorPID.setDesiredVelocity(speed*(1+differential));
				}else{
					float angleRatio = 0.0013*(middleServo-angle);
					float differential = angleRatio/(2-angleRatio);
					left_motorPID.setDesiredVelocity(speed*(1+differential));
					right_motorPID.setDesiredVelocity(speed*(1-differential));
				}

				encoderLval = LEncoder.GetCount();
				encoderRval = -REncoder.GetCount();

				if (turn_on_motor){
					powerR = right_motorPID.getPID();
					powerL = left_motorPID.getPID();
					if (abs(powerL)>400 || abs(powerR)>400){
						powerL = 400;
						powerR = 400;
					}
					if (powerR > 0){
						right_motor.SetClockwise(false);
						right_motor.SetPower(powerR);
					}else{
						right_motor.SetClockwise(true);
						right_motor.SetPower(-powerR);
					}
					if (powerL > 0){
						left_motor.SetClockwise(true);
						left_motor.SetPower(powerL);
					}else{
						left_motor.SetClockwise(false);
						left_motor.SetPower(-powerL);
					}

				}else{
					left_motorPID.setDesiredVelocity(0);
					right_motorPID.setDesiredVelocity(0);
					left_motor.SetPower(0);
					right_motor.SetPower(0);
					if (encoderLval != 0){
						LEncoder.Update();
					}
					if (encoderRval != 0){
						REncoder.Update();
					}
				}



#endif

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
					sprintf(c,"Sv:%d ", servo.GetDegree());\
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
					sprintf(c,"l_en:%d ", (int)encoderLval);
					writer.WriteBuffer(c,10);
					for(int i=0; i<10; i++){
						c[i] = ' ';
					}
					lcd.SetRegion(Lcd::Rect(0,135,88,15));
					sprintf(c,"r_en:%d ", (int)encoderRval);
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

				cycleTime = System::Time()-lastTime;
			}
		}
    }
    return 0;
}
