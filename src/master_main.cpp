/*
 * master_main.cpp
 *
 *  Created on: Jun 21, 2018
 *      Author: morristseng
 */

#define Master

#ifdef Master

#include <cmath>
#include <vector>
#include <cassert>
#include <cstring>
#include <string>
#include <libbase/k60/mcg.h>
#include "libbase/k60/vectors.h"
#include "libbase/k60/pit.h"
#include "libbase/k60/adc.h"
#include "libsc/k60/jy_mcu_bt_106.h"
#include <libsc/k60/ov7725.h>
#include <libsc/k60/ov7725_configurator.h>
#include <libsc/system.h>
#include <libsc/led.h>
#include "libsc/led.h"
#include "libsc/joystick.h"
#include "libsc/st7735r.h"
#include "libsc/battery_meter.h"
#include "libsc/lcd_typewriter.h"
#include "libsc/passive_buzzer.h"
#include "libsc/battery_meter.h"
#include "config.h"
#include "PID.h"
#include "corner.h"
#include "edge.h"
#include "facing.h"
#include "bluetooth.h"
#include "find_midline.h"
#include "DualCar_UART.h"
#include "mag_func.h"
#include "useful_functions.h"
#include "menu.h"
#include "BoardID.h"
#include "DistanceModule.h"

#define pi 3.1415926

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

char *str = "";

//magnetic
int max(int a, int b) {
	return a > b ? a : b;
}
int min(int a, int b) {
	return a < b ? a : b;
}

//camera
#define Width 80
#define Height 60
bool camptr[Height][Width];

int stop = 0;
int mode = 0;

inline bool ret_cam_bit(int x, int y, const Byte* camBuffer) {
	return ((camBuffer[y * 10 + x / 8] >> (7 - (x % 8))) & 1); //return 1 if black
}

//main
int main() {
	System::Init();

	BoardID board;

	Ov7725 camera(myConfig::getCameraConfig(Width, Height));
	camera.Start();

	Led led0(myConfig::GetLedConfig(0));
	Led led1(myConfig::GetLedConfig(1));
	Led led2(myConfig::GetLedConfig(2));
	Led led3(myConfig::GetLedConfig(3));

	led0.SetEnable(1);
	led1.SetEnable(1);
	led2.SetEnable(1);
	led3.SetEnable(1);

	Mag mag(board.isCar1());

	BatteryMeter batteryMeter(myConfig::GetBatteryMeterConfig());
	float batteryVoltage = batteryMeter.GetVoltage();
	PassiveBuzzer::Config config;
	PassiveBuzzer buzz(config);
	buzz.SetNote(522);
	buzz.SetBeep(batteryVoltage < 7.4);

	FutabaS3010 servo(myConfig::GetServoConfig());
	AlternateMotor right_motor(myConfig::GetMotorConfig(1));
	AlternateMotor left_motor(myConfig::GetMotorConfig(0));
	St7735r lcd(myConfig::GetLcdConfig());
	LcdTypewriter writer(myConfig::GetWriterConfig(&lcd));
	LcdConsole console(myConfig::GetConsoleConfig(&lcd));

	DirEncoder encoderR(myConfig::GetEncoderConfig(0));
	DirEncoder encoderL(myConfig::GetEncoderConfig(1));
	PID servoPIDStraight(2200, 0.03);
	PID servoPIDCurve(5500, 1); //4825,0.5
	PID servoPIDAlignCurve(-6, 1);
    PID servoPIDAlignStraight(-6,1);
    PID cameraPID(0,0);
	PID left_motorPID(0, 0, 0, &encoderL, false);
	PID right_motorPID(0, 0, 0, &encoderR, true);

	//pid value

	float left_motor_pid[3] = { 0.0, 0.0, 0.0 };
	float right_motor_pid[3] = { 0.0, 0.0, 0.0 };
	float straight_servo_pd[2] = { 5500, 300000 };
	float curve_servo_pd[2] = { 9300, 230000 };
	float align_servo_pd[2] = {-6, 1};
	float camera_pd[2] = {80,0};
	bool forwardL = true, forwardR = true;
	uint16_t middleServo = 1035, leftServo = 1305, rightServo = 750; // originally const


	if (board.isCar1()) {
		left_motor_pid[0] = 0.06;
		left_motor_pid[1] = 0.002;
		left_motor_pid[2] = 0.008;

		right_motor_pid[0] = 0.05;
		right_motor_pid[1] = 0.001;
		right_motor_pid[2] = 0.008;

		straight_servo_pd[0] = 5500;
		straight_servo_pd[1] = 200000;

		curve_servo_pd[0] = 10500;
		curve_servo_pd[1] = 850000;

		align_servo_pd[0] = -6;
		align_servo_pd[1] = 1;

		forwardL = true;
		forwardR = true;

		middleServo = 1045;
		leftServo = 1305;
		rightServo = 760;

		mag.SetMag(1);
	} else {
		left_motor_pid[0] = 0.06;
		left_motor_pid[1] = 0.002;
		left_motor_pid[2] = 0.008;

		right_motor_pid[0] = 0.07;
		right_motor_pid[1] = 0.002;
		right_motor_pid[2] = 0.008;

		straight_servo_pd[0] = 5500;
		straight_servo_pd[1] = 200000;

		curve_servo_pd[0] = 10000;
		curve_servo_pd[1] = 950000;

		align_servo_pd[0] = -6;
		align_servo_pd[1] = 1;

		forwardL = false;
		forwardR = true;

		middleServo = 840;
		leftServo = 1150;
		rightServo = 560;

		mag.SetMag(2);

	}
//#ifdef car1
//	float left_motor_pid[3] = { 0.036, 0.003, 0.0 };
//	float right_motor_pid[3] = { 0.034, 0.004, 0.0 };
//	float straight_servo_pd[2] = { 5500, 300000 };
//	float curve_servo_pd[2] = { 9300, 230000 };
//	bool forwardL = true, forwardR = true;
//	const uint16_t middleServo = 1035, leftServo = 1305, rightServo = 750;
//	mag.SetMag(1);
//#endif
//#ifdef car2
//	float left_motor_pid[3] = { 0.036, 0.003, 0.00004};
//	float right_motor_pid[3] = { 0.036, 0.003, 0.00004 };
//	float straight_servo_pd[2] = { 5300, 350000 };
//	float curve_servo_pd[2] = { 10500, 200000 };
//	bool forwardL = false, forwardR = true;
//	const uint16_t middleServo = 850, leftServo = 1150, rightServo = 550;
//	mag.SetMag(2);
//#endif

	typedef enum {
		normal = 0,
		leave,
		align,
		side,
		back
	} carState;
	carState state = normal;
	uint32_t lastTime = 0, approachTime = 0;
	bool approaching = false, cali = false;

	float angle = middleServo;
	float camera_angle = middleServo;
	float lastServo = 0;

	uint8_t cycleTime = 0;
	const uint8_t cycle = 12;
	float loopSpeed = 4 * cycle, highSpeed = 6 * cycle, alignSpeed = 6 * cycle;
	float speed = highSpeed;
	float encoderLval, encoderRval;
	float voltL, voltR;
	int32_t powerL, powerR;

	// below sync data to the computer side
	DualCar_UART uart0(1); // << BT related

	//allignent
//	uart0.add(DualCar_UART::FLOAT::f0, &align_servo_pd[0], false);
//	uart0.add(DualCar_UART::FLOAT::f1, &align_servo_pd[1], false);
	uart0.add(DualCar_UART::BOOLEAN::b0, &approaching, true);

	// motor & servo pid
	uart0.add(DualCar_UART::FLOAT::f0, &left_motor_pid[0], false);
	uart0.add(DualCar_UART::FLOAT::f1, &left_motor_pid[1], false);
	uart0.add(DualCar_UART::FLOAT::f2, &left_motor_pid[2], false);

	uart0.add(DualCar_UART::FLOAT::f3, &right_motor_pid[0], false);
	uart0.add(DualCar_UART::FLOAT::f4, &right_motor_pid[1], false);
	uart0.add(DualCar_UART::FLOAT::f5, &right_motor_pid[2], false);

	uart0.add(DualCar_UART::FLOAT::f6, &straight_servo_pd[0], false);
	uart0.add(DualCar_UART::FLOAT::f7, &straight_servo_pd[1], false);

	uart0.add(DualCar_UART::FLOAT::f8, &curve_servo_pd[0], false);
	uart0.add(DualCar_UART::FLOAT::f9, &curve_servo_pd[1], false);

	uart0.add(DualCar_UART::FLOAT::f12, &encoderLval, true);
	uart0.add(DualCar_UART::FLOAT::f13, &encoderRval, true);

//
	uart0.parseValues();

	DistanceModule UltrasonicSensor([](float distanceInCm) {
		if (distanceInCm < 30) {
			// what to do when it's below 30 cm
		}
	});

	//joystick value
	DualCar_Menu menu;
	Mode mode0(0);
	Mode mode1(1);
	Mode mode2(2);
	Mode mode3(3);
	Mode ClearMode(4);
	//
	//dotted lines
	uint32_t accumulate_corner = 0;
	uint8_t dot_time = 0;
	bool is_dot_line = false;
	bool start_count_corner = false;

	// for loop
	float loop_control_const = 85;
	bool right_loop = false;
	bool left_loop = false;
	bool camera_control = false;//true for camera, false for mag
	bool loop_phase[5] = {false, false, false, false, false};

	//printing change value
//	Items item5("sr_kp", &straight_servo_pd[0], true);
//	Items item6("sr_kd", &straight_servo_pd[1], true);
//	Items item7("cr_kp", &curve_servo_pd[0], true);
//	Items item8("cr_kd", &curve_servo_pd[1], true);
//	item5.set_increment(100);
//	item6.set_increment(100);
//	item7.set_increment(100);
//	item8.set_increment(100);



	Joystick js(myConfig::GetJoystickConfig(Joystick::Listener([&]
	(const uint8_t id, const Joystick::State state) {
		if(state == Joystick::State::kRight) {
			menu.change_mode(true);
		}
		else if(state == Joystick::State::kLeft){
			menu.change_mode(false);
		}
		else if(state == Joystick::State::kSelect){
			menu.select_pressed();
		}
		else if(state == Joystick::State::kUp){
			menu.change_line(true);
		}
		else if(state == Joystick::State::kDown){
			menu.change_line(false);
		}
	})));

	servo.SetDegree(middleServo);

	//bluetooth communicate
	vector<pair<int, int>> slave_edge;
	vector<pair<int, int>> master_edge;
	vector<pair<int, int>> midline;
	M_Bluetooth m_master_bluetooth;


	int32_t on9lastSent = 0;
	int32_t on9lastMain = 0;
	float x = 0;
	vector<Corner> slave_corner;


	int straightCounter = 0;
	int curveCounter = 0;
	bool isStraight = true;
	float middle_slope = 0;
	bool adjust_midline_slope = false;


	while (1) {
		if (System::Time() != lastTime) {

			lastTime = System::Time();

//			 bt send motor speed
			if (lastTime - on9lastSent > 50) {
				on9lastSent = lastTime;
				uart0.Send_float(DualCar_UART::FLOAT::f10, left_motorPID.getcurrentVelocity());
				uart0.Send_float(DualCar_UART::FLOAT::f11, right_motorPID.getcurrentVelocity());
			}

			uart0.RunEveryMS();
			mag.TakeSample();
			if (lastTime - on9lastMain >= cycle) {
				x += 0.02;
				dot_time++;
				on9lastMain = lastTime;
				const Byte* camBuffer = camera.LockBuffer();
				camera.UnlockBuffer();
				mag.Update();
				if (cali) {
					mag.Calibrate();
				}
				if (mag.noMagField() && !approaching && menu.get_mode() == DualCar_Menu::Page::kStart && menu.get_selected()) {
					left_motorPID.setDesiredVelocity(0);
					right_motorPID.setDesiredVelocity(0);
					menu.select_pressed();
				}
				//for alignment
				if (state == normal && approaching) {
					state = leave;
					speed = alignSpeed;
					approachTime = lastTime;
				} else if (state == leave && mag.SmallerThanE(0, 0.5)) {
					state = align;
				} else if (state == align && mag.Difference(0, 1) < 10 && abs((int) angle - middleServo) < 50) {
					state = side;
				} else if (state == side && !approaching) {
					state = normal;
					lastServo = -100;
//				} else if (state == back && !mag.SmallerThanE(0, 1)) {
//					state = normal;
//					speed = 0;
//					speed = highSpeed;
				}

				if (lastTime-approachTime > 1000 && approaching){
					approaching = false;
					approachTime = lastTime;
				}

				servoPIDStraight.setkP(straight_servo_pd[0]);
				servoPIDStraight.setkD(straight_servo_pd[1]);
				servoPIDCurve.setkP(curve_servo_pd[0]);
				servoPIDCurve.setkD(curve_servo_pd[1]);
				servoPIDAlignCurve.setkP(align_servo_pd[0]);
				servoPIDAlignCurve.setkD(align_servo_pd[1]);
				cameraPID.setkP(camera_pd[0]);
				cameraPID.setkD(camera_pd[1]);
				left_motorPID.setkP(left_motor_pid[0]);
				left_motorPID.setkI(left_motor_pid[1]);
				left_motorPID.setkD(left_motor_pid[2]);
				right_motorPID.setkP(right_motor_pid[0]);
				right_motorPID.setkI(right_motor_pid[1]);
				right_motorPID.setkD(right_motor_pid[2]);

				vector<Corner> master_corner;
				master_corner = check_corner(camBuffer, 30, 60, true, master_edge);
				slave_corner = m_master_bluetooth.get_slave_corner();

				//alignment
				if(((master_corner.size()>0)&&(slave_corner.size()>0))&&(start_count_corner==false)){
					dot_time = 0;
					start_count_corner = true;
				}

				if(start_count_corner){
					if(dot_time==10){
						start_count_corner = false;
						dot_time = 0;
						if(accumulate_corner>6){
							is_dot_line = true;
							led0.SetEnable(false);
							buzz.SetNote(440);
							buzz.SetBeep(true);
							if(!approaching && (lastTime - approachTime >= 10000 || approachTime == 0))
								approaching = true;
						}
						else{
							is_dot_line = false;
							led0.SetEnable(true);
//							buzz.SetBeep(false);
						}
						accumulate_corner = 0;
					}
					else{
						accumulate_corner += master_corner.size();
						accumulate_corner += slave_corner.size();
					}
				}
				else{
					if(dot_time==10){
						dot_time = 0;
					}
				}
				//


				//loop

				float master_slope = 0;
				float slave_slope = 0;
				int slave_edge_size = m_master_bluetooth.get_edge_size();
				master_slope = find_slope(master_edge);
				slave_slope = m_master_bluetooth.get_m_slope();
				if((mag.BigMag())&&(loop_phase[0]==false)&&(loop_phase[1]==false)&&(loop_phase[2]==false)&&(loop_phase[3]==false)){
					loop_phase[0] = true;
					if(master_edge.size() > slave_edge_size)
						right_loop = true;
					else
						right_loop = false;
				}
				if(loop_phase[0]==true){
					if(right_loop){
						if((slave_edge_size>0)&&(slave_slope<0.7)){
							buzz.SetNote(523);
							buzz.SetBeep(true);
							middle_slope = master_slope;
							loop_phase[0] = false;
							loop_phase[1] = true;
						}
					}
					else{
						if((master_edge.size()>0)&&(master_slope>0.7)){//the slope when the car is place on the biggest loop's middle
							buzz.SetNote(523);
							buzz.SetBeep(true);
							middle_slope = master_slope;
							loop_phase[0] = false;
							loop_phase[1] = true;
						}
					}
				}
				else if(loop_phase[1] == true){
					if(right_loop){
					}
					else{
						camera_control = true;
						camera_angle = middle_slope*150;//car 1:150
						camera_angle += middleServo;
						if(slave_edge_size<2){
							loop_phase[1] = false;
							loop_phase[2] = true;
						}
					}
				}
				else if(loop_phase[2] == true){
					if(right_loop){

					}
					else{
						if((slave_edge_size>0)||(slave_corner.size()==1)){
							buzz.SetBeep(false);
							loop_phase[2] = false;
							loop_phase[3] = true;
						}
					}
				}
				else if(loop_phase[3] == true){
					if(right_loop){

					}
					else{
						camera_control = false;
						right_loop = false;
						loop_phase[3] = false;
					}
				}


//				if ((mag.BigMag())&&(right_loop==false)&&(left_loop == false)){
//					if(master_edge.size() > slave_edge_size){
//						right_loop = true;
//						left_loop = false;
//					}
//					else{
//						right_loop = false;
//						left_loop = true;
//					}
//				}else{
//
//				}
//
//				if(right_loop){
//					if((master_corner.size()+slave_corner.size())==1){
//						camera_control = false;
//						right_loop = false;
//					}
//				}
//
//				else if(left_loop){
//					if(master_edge.size()>0){
////						master_slope = find_slope(master_edge);
//						if((master_slope>0.7)&&(!adjust_midline_slope)){//car 1 0.7
//							if((left_loop)){
//								buzz.SetNote(523);
//								buzz.SetBeep(true);
//								middle_slope = master_slope;
//								adjust_midline_slope = true;
//							}
//						}
//					}
//					if((adjust_midline_slope)){
//							camera_control = true;
//							camera_angle = middle_slope*150;//car 1:150
//							camera_angle += middleServo;
//					}
//
//
//					if(((slave_corner.size()==1))&&adjust_midline_slope&&(camera_control)){
//						buzz.SetBeep(false);
//						camera_control = false;
//						left_loop = false;
//					}
//				}
//
//				else{
//					buzz.SetBeep(false);
//					adjust_midline_slope = false;
//				}
//


				if (cali || menu.get_mode() < DualCar_Menu::Page::kMag) {
					angle = 0;
				} else if (state == normal) {
//					angle = servoPIDAlignCurve.getPID(mag.GetEMin(0)*mag.GetMulti(0), mag.GetMag(0));
					if (mag.SmallerThanMin(0, 1.5) || mag.SmallerThanMin(1, 1.5)){
						angle = lastServo;
						if(!approaching){
//							buzz.SetNote(800);
//							buzz.SetBeep(true);
						}
					} else {
						angle = servoPIDCurve.getPID(0.0, mag.GetLinear(0));
						lastServo = angle;
						if(!approaching){
//							buzz.SetNote(520);
//							buzz.SetBeep(true);
						}
					}
				} else if (state == leave){
					angle = leftServo;
//					angle = servoPIDAlignCurve.getPID(mag.GetMin(0)*mag.GetMulti(0), mag.GetMag(0));
				} else if (state == align) {
					angle = servoPIDAlignCurve.getPID(mag.GetEMin(0)*mag.GetMulti(0), mag.GetMag(1));
				} else if (state == side){
					angle = servoPIDAlignStraight.getPID(mag.GetEMin(0)*mag.GetMulti(0), mag.GetMag(1));
				} else if (state == back){
					angle = servoPIDAlignCurve.getPID(mag.GetEMax(0)*mag.GetMulti(0), mag.GetMag(0));
				}
				m_master_bluetooth.reset_m_edge();


				angle += middleServo;
				angle = max(rightServo, min(leftServo, angle));


				if(!camera_control){
					servo.SetDegree(angle);
				}
				else{
					servo.SetDegree(camera_angle);
				}

				if (angle > middleServo) {
					float angleRatio = 0.0013 * (angle - middleServo);
					float differential = angleRatio / (2 - angleRatio);
					left_motorPID.setDesiredVelocity(speed * (1 - differential));
					right_motorPID.setDesiredVelocity(speed * (1 + differential));
//					left_motorPID.setDesiredVelocity(speed * sin(x));
//					right_motorPID.setDesiredVelocity(speed * sin(x));
//					left_motorPID.setDesiredVelocity(speed);
//					right_motorPID.setDesiredVelocity(speed);
				} else {
					float angleRatio = 0.0013 * (middleServo - angle);
					float differential = angleRatio / (2 - angleRatio);
					left_motorPID.setDesiredVelocity(speed * (1 + differential));
					right_motorPID.setDesiredVelocity(speed * (1 - differential));
//					left_motorPID.setDesiredVelocity(speed * sin(x));
//					right_motorPID.setDesiredVelocity(speed * sin(x));
//					left_motorPID.setDesiredVelocity(speed);
//					right_motorPID.setDesiredVelocity(speed);
				}

				if (menu.get_mode() != DualCar_Menu::Page::kStart) {
					encoderL.Update();
					encoderR.Update();
					encoderLval = encoderL.GetCount();
					encoderRval = -encoderR.GetCount();
				}

				if (menu.get_mode() == DualCar_Menu::Page::kStart){
					if (menu.get_selected()) {
						voltR = right_motorPID.getPID();
						voltL = left_motorPID.getPID();
						powerR = voltR/batteryVoltage*1000;
						powerL = voltL/batteryVoltage*1000;
						if (powerR > 0) {
							right_motor.SetClockwise(forwardR);
							right_motor.SetPower(powerR);
						} else {
							right_motor.SetClockwise(!forwardR);
							right_motor.SetPower(-powerR);
						}
						if (powerL > 0) {
							left_motor.SetClockwise(forwardL);
							left_motor.SetPower(powerL);
						} else {
							left_motor.SetClockwise(!forwardL);
							left_motor.SetPower(-powerL);
						}
					} else {
						left_motorPID.setDesiredVelocity(0);
						right_motorPID.setDesiredVelocity(0);
						left_motor.SetPower(0);
						right_motor.SetPower(0);
						if (encoderLval != 0) {
							encoderL.Update();
						}
						if (encoderRval != 0) {
							encoderR.Update();
						}
					}
				} else{//not start, printing show value
					char *s = "";
					if (board.isCar1()) {
						s = "Master car1";
					} else {
						s = "Master car2";
					}

					Items item0(s);
					Items item1 ("M_sl", master_slope);
					Items item2("S_sl", m_master_bluetooth.get_m_slope());
					Items item3("Sv", servo.GetDegree());
					Items item4("M_cor", master_corner.size());
					Items item5("S_cor", slave_corner.size());

					Items item6("cam_con", camera_control);
					Items item7("r_loop", right_loop);
					Items item8("p0", loop_phase[0]);
					Items item9("p1", loop_phase[1]);
					Items item10("p2", loop_phase[2]);
					Items item26("p3", loop_phase[3]);
					Items item24("M_es",master_edge.size());
					Items item25("S_es", slave_edge_size);

					Items item11("speed", speed);
					Items item12("r_en", encoderRval);
					Items item13("l_en", encoderLval);
					Items item14("lines", menu.get_line());
					Items item15("selected", menu.get_selected());



					Items item16("left", mag.GetMag(0));
					Items item17("right", mag.GetMag(1));
					Items item18("", dot_time);
					Items item19("volt", batteryMeter.GetVoltage());
					Items item20("state", (int)state);
					Items item21("l", mag.GetLinear(0));
					Items item22("Ultra", UltrasonicSensor.getDistance());
					Items item23("ac_cor", accumulate_corner);


					mode0.add_items(&item0);
					mode0.add_items(&item1);
					mode0.add_items(&item2);
					mode0.add_items(&item3);
					mode0.add_items(&item4);
					mode0.add_items(&item5);

					mode1.add_items(&item6);
					mode1.add_items(&item7);
					mode1.add_items(&item8);
					mode1.add_items(&item9);
					mode1.add_items(&item10);
					mode1.add_items(&item26);
					mode1.add_items(&item24);
					mode1.add_items(&item25);

					mode2.add_items(&item11);
					mode2.add_items(&item12);
					mode2.add_items(&item13);
					mode2.add_items(&item14);
					mode2.add_items(&item15);

					mode3.add_items(&item16);
					mode3.add_items(&item17);
					mode3.add_items(&item18);
					mode3.add_items(&item19);
					mode3.add_items(&item20);
					mode3.add_items(&item21);
					mode3.add_items(&item22);
					mode3.add_items(&item23);
				}

				menu.add_mode(&mode0);
				menu.add_mode(&mode1);
				menu.add_mode(&mode2);
				menu.add_mode(&mode3);
				menu.add_mode(&ClearMode);


				if (menu.get_mode() == DualCar_Menu::Page::kImage) {
					if (menu.change_screen()) {
						lcd.Clear();
					}
					lcd.SetRegion(Lcd::Rect(0, 0, Width, Height));
					lcd.FillBits(0x0000, 0xFFFF, camBuffer, Width * Height);
					for (int i = 0; i < master_edge.size(); i++) {
						lcd.SetRegion(
								Lcd::Rect(master_edge[i].first,
										master_edge[i].second, 2, 2));
						lcd.FillColor(Lcd::kRed);
					}
					for (int i = 0; i < slave_edge.size(); i++) {
						lcd.SetRegion(
								Lcd::Rect(slave_edge[i].first, slave_edge[i].second, 2, 2));
						lcd.FillColor(Lcd::kPurple);
					}

					for (int i = 0; i < midline.size(); i++) {
						lcd.SetRegion(
								Lcd::Rect(midline[i].first, midline[i].second,
										2, 2));
						lcd.FillColor(Lcd::kBlue);
					}

					for (int i = 0; i < master_corner.size(); i++) {
						lcd.SetRegion(
								Lcd::Rect(master_corner[i].get_xcoord(), master_corner[i].get_ycoord(), 2, 2));
						lcd.FillColor(Lcd::kGreen);
					}

					for (int i = 0; i < slave_corner.size(); i++) {
						lcd.SetRegion(
								Lcd::Rect(slave_corner[i].get_xcoord(), slave_corner[i].get_ycoord(), 2, 2));
						lcd.FillColor(Lcd::kGreen);
					}

					for(int i=0; i<menu.m_menu[menu.get_mode()]->get_max_line(); i++){
						lcd.SetRegion(Lcd::Rect(0, 60+15*i, 88, 15));
						writer.WriteBuffer(menu.m_menu[menu.get_mode()]->m_items[i]->get_message(),15);
					}
				}

				else if (menu.get_mode() != DualCar_Menu::Page::kStart) {
					if (menu.change_screen()) {
						lcd.Clear();
					}
					for(int i=0; i<menu.m_menu[menu.get_mode()]->get_max_line(); i++){
						lcd.SetRegion(Lcd::Rect(0, 15*i, 88, 15));
						writer.WriteBuffer(menu.m_menu[menu.get_mode()]->m_items[i]->get_message(),15);
					}
				}
				else{
					if (menu.change_screen()) {
						lcd.Clear();
					}
				}
				menu.clear();
				master_edge.clear();
				slave_edge.clear();
				midline.clear();
				master_corner.clear();
				cycleTime = System::Time() - lastTime;
			}
		}
	}
	return 0;
}

#endif
