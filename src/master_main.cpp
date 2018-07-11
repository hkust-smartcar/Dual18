/*
 * master_main.cpp
 *
 *  Created on: Jun 21, 2018
 *      Author: morristseng
 */

#define Master
//combined
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

int mode = 0;

inline bool ret_cam_bit(int x, int y, const Byte* camBuffer) {
	return ((camBuffer[y * 10 + x / 8] >> (7 - (x % 8))) & 1); //return 1 if black
}

typedef enum {
	kNormal = 0,
	kLeave,
	kStop,
	kAlign,
	kSide,
	kLoop,
	kExitLoop,
	kLessTurn
} carState;
carState magState = kNormal;

static const uint8_t cycle = 12;
static float loopSpeed = 7 * cycle, highSpeed = 9 * cycle, alignSpeed = 7 * cycle;
static float speed = highSpeed;

uint8_t loop_control(uint8_t state, bool &is_loop, Mag* magnetic, bool &left_loop, bool &camera_control, float &camera_angle, int master_edge_size, int slave_edge_size, int m_edge_xmid, int s_edge_xmid, int master_corner_size, int slave_corner_size, float master_slope, float slave_slope){
	static bool left;
	left_loop = left;
	static bool cameraReady = false, magReady = false;
	switch(state){
	case 0:
		if(is_loop){
			state = 1;
		}
		break;
	case 1:
		if(master_slope<(-1*slave_slope))//master_slope<magnetic->isRightLoop()
			left = true;
		else
			left = false;
		state = 2;
		break;
	case 2:
		if(left){
			if(master_edge_size>20)
				state = 3;
		}else{
			if(slave_edge_size>20)
				state = 3;
		}
		break;

	case 3:
		if(left){
			if(((master_slope>0.5)&&(m_edge_xmid<40))||(master_edge_size<15))
				state = 4;
		}else{
			if(((slave_slope<-0.5)&&(s_edge_xmid>40))||(slave_edge_size<15))
				state = 4;
		}
		break;
	case 4:
		if(left){
			camera_control = true;
			float difference = (40 - m_edge_xmid)/40.0;
			if(difference<0.5){
				camera_angle = difference*100;
			}
			else{
				camera_angle = difference*200;
			}
			if(slave_edge_size<4){
				state = 5;
				cameraReady = false;
				magReady = false;
			}
		}else{
			camera_control = true;
			float difference = (40 - s_edge_xmid)/40.0;
			if(difference>-0.5){
				camera_angle = difference*100;
			}
			else{
				camera_angle = difference*200;
			}
			if(master_edge_size<4){
				state = 5;
				cameraReady = false;
				magReady = false;
			}
		}
		break;
	case 5:
		if(left){
			float difference = (40 - m_edge_xmid)/40.0;
			if(difference<0.5){
				camera_angle = difference*250;
			}
			else{
				camera_angle = difference*400;
			}
			if((slave_edge_size>5)||(slave_corner_size==1)){
				cameraReady = true;
			}
			if (magnetic->GetXSum() < 120){
				magReady = true;
			}
			if(cameraReady && magReady){
				state = 6;
				camera_control = false;
				magState = carState::kLoop;
			}
		}
		else{
			float difference = (40 - s_edge_xmid)/40.0;
			if(difference>-0.5){
				camera_angle = difference*250;
			}
			else{
				camera_angle = difference*400;
			}
			if((master_edge_size>5)||(master_corner_size==1)){
				state = 6;
				camera_control = false;
				magState = carState::kLoop;
			}
		}
		break;
	case 6:
		if(magnetic->outLoop()){
			state = 7;
			magState = carState::kExitLoop;
			speed = loopSpeed;
		}
		break;
	case 7:
		if (magnetic->isMidLoop()){
			state = 8;
			magState = carState::kLessTurn;
		}
		break;
	case 8:
		if (!magnetic->isLoop() && magnetic->isBigStraight()){
			state = 0;
			magState = carState::kNormal;
			speed = highSpeed;
			is_loop = false;
		}
		break;
	default:
		break;
	}
	return state;
}



//main
int main() {
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

	Mag mag;

	BatteryMeter batteryMeter(myConfig::GetBatteryMeterConfig());
	float batteryVoltage = batteryMeter.GetVoltage();
	PassiveBuzzer::Config config;
	PassiveBuzzer buzz(config);
	buzz.SetNote(522);
	buzz.SetTexture(1);
	buzz.SetBeep(batteryVoltage < 7.4);

	FutabaS3010 servo(myConfig::GetServoConfig());
	AlternateMotor right_motor(myConfig::GetMotorConfig(1));
	AlternateMotor left_motor(myConfig::GetMotorConfig(0));
	St7735r lcd(myConfig::GetLcdConfig());
	LcdTypewriter writer(myConfig::GetWriterConfig(&lcd));
	LcdConsole console(myConfig::GetConsoleConfig(&lcd));

	DirEncoder encoderR(myConfig::GetEncoderConfig(0));
	DirEncoder encoderL(myConfig::GetEncoderConfig(1));
	PID servoPIDLoop(14500, 880000);
	PID servoPIDx(5500, 1);
	PID servoPIDy(500, 1);
	PID servoPIDAlignCurve(-6, 1);
	PID cameraPID(0,0);
	PID left_motorPID(0, 0, 0, &encoderL, false);
	PID right_motorPID(0, 0, 0, &encoderR, true);

	Edge left_edge(false);

	//pid value

	float left_motor_pid[3] = { 0.0, 0.0, 0.0 };
	float right_motor_pid[3] = { 0.0, 0.0, 0.0 };
	float loop_servo_pd[2] = { 5500, 300000 };
	float x_servo_pd[2] = { 9300, 230000 };
	float y_servo_pd[2] = { 9300, 230000 };
	float align_servo_pd[2] = {-6, 1};
	float camera_pd[2] = {80,0};
	bool forwardL = true, forwardR = true;
	uint16_t middleServo = 1035, leftServo = 1305, rightServo = 750; // originally const

	float loop_slope;
	int loopLsmall, loopLbig, loopRsmall, loopRbig;

	if (board.isCar1()) {
	    left_motor_pid[0] = 0.07;
	    left_motor_pid[1] = 0.01;
	    left_motor_pid[2] = 0.02;

	    right_motor_pid[0] = 0.05;
	    right_motor_pid[1] = 0.009;
	    right_motor_pid[2] = 0.015;

		loop_servo_pd[0] = 14500;
		loop_servo_pd[1] = 400000;

		x_servo_pd[0] = 9702;
		x_servo_pd[1] = 516780;

		y_servo_pd[0] = 2.5885959;
		y_servo_pd[1] = 217.845;

		align_servo_pd[0] = -7.5;
		align_servo_pd[1] = -130;

		forwardL = true;
		forwardR = true;

		middleServo = 1045;
		leftServo = 1330;
		rightServo = 695;

		mag.InitMag(1);

		loop_slope = 0.6;
		loopRsmall = 600;
		loopRbig = 650;
		loopLsmall = 350;
		loopLbig = 400;

	} else {
	    left_motor_pid[0] = 0.03;
	    left_motor_pid[1] = 0.0055;
	    left_motor_pid[2] = 0.007;

	    right_motor_pid[0] = 0.035;
	    right_motor_pid[1] = 0.008;
	    right_motor_pid[2] = 0.007;

	    loop_servo_pd[0] = 14500;
	    loop_servo_pd[1] = 350000;

	    x_servo_pd[0] = 11200;
	    x_servo_pd[1] = 840000;

	    y_servo_pd[0] = 11200;
	    y_servo_pd[1] = 840000;

		align_servo_pd[0] = -5;
		align_servo_pd[1] = -17;

		forwardL = false;
		forwardR = true;

		middleServo = 840;
		leftServo = 1130;
		rightServo = 560;

		mag.InitMag(2);

		loop_slope = 0.6;
		loopRsmall = 450;
		loopRbig = 600;
		loopLsmall = 450;
		loopLbig = 600;
	}

	uint32_t lastTime = 0, approachTime = 0;
	bool approaching = false, isFirst = false, firstArrived = false, secondArrived = false, USsent = false;
	bool cali = false;
	bool cantDetect = false;

	float angle = middleServo;
	float camera_angle = middleServo;
	float lastServo = 0;

	uint8_t cycleTime = 0;
	float encoderLval, encoderRval;
	float voltL, voltR;
	int32_t powerL, powerR;
	float batterySum = 0, angleSum = 0;
	uint16_t batteryCount = 0, angleCount = 0;


	// below sync data to the computer side
	DualCar_UART uart0(1); // << BT related

	// joseph: chnaged the 3 below to false, better send implicitly
	uart0.add(DualCar_UART::BOOLEAN::b0, &approaching, false);
	uart0.add(DualCar_UART::BOOLEAN::b1, &firstArrived, false);
	uart0.add(DualCar_UART::BOOLEAN::b2, &secondArrived, false);
	uart0.add(DualCar_UART::BOOLEAN::b3, &isFirst, false);
	uart0.add(DualCar_UART::BOOLEAN::b4, &USsent, false);


	// motor & servo pid
	uart0.add(DualCar_UART::FLOAT::f0, &left_motor_pid[0], false);
	uart0.add(DualCar_UART::FLOAT::f1, &left_motor_pid[1], false);
	uart0.add(DualCar_UART::FLOAT::f2, &left_motor_pid[2], false);

	uart0.add(DualCar_UART::FLOAT::f3, &right_motor_pid[0], false);
	uart0.add(DualCar_UART::FLOAT::f4, &right_motor_pid[1], false);
	uart0.add(DualCar_UART::FLOAT::f5, &right_motor_pid[2], false);

	uart0.add(DualCar_UART::FLOAT::f6, &loop_servo_pd[0], false);
	uart0.add(DualCar_UART::FLOAT::f7, &loop_servo_pd[1], false);

	uart0.add(DualCar_UART::FLOAT::f8, &x_servo_pd[0], false);
	uart0.add(DualCar_UART::FLOAT::f9, &x_servo_pd[1], false);

	uart0.add(DualCar_UART::FLOAT::f16, &y_servo_pd[0], false);
	uart0.add(DualCar_UART::FLOAT::f17, &y_servo_pd[1], false);

	uart0.add(DualCar_UART::FLOAT::f14, &align_servo_pd[0], false);
	uart0.add(DualCar_UART::FLOAT::f15, &align_servo_pd[1], false);

	uart0.add(DualCar_UART::FLOAT::f12, &encoderLval, true);
	uart0.add(DualCar_UART::FLOAT::f13, &encoderRval, true);

	uart0.add(DualCar_UART::FLOAT::f20, &speed, false);

	uart0.parseValues();


	DistanceModule UltrasonicSensor([&](float distanceInCm) {
		if (distanceInCm < 30) {
			if(approaching && secondArrived){
				approaching = false;
				firstArrived = false;
				secondArrived = false;
				isFirst = false;
				uart0.Send_bool(DualCar_UART::BOOLEAN::b4, true);
				approachTime = lastTime;
			}
		}
	});

	//joystick value
	DualCar_Menu menu;
	Mode mode0(0);
	Mode mode1(1);
	Mode mode2(2);
	Mode mode3(3);
	Mode mode4(4);
	Mode ClearMode(5);
	//
	//dotted lines
	uint32_t accumulate_corner = 0;
	uint8_t dot_time = 0;
	bool is_dot_line = false;
	bool start_count_corner = false;
	bool rubbishJoseph = false;//two car

	//for loop ver2
	float loop_control_const = 85;
	bool right_loop = false;
	bool left_loop = false;
	bool in_loop = false;
	uint8_t loop_phase = 0;
	bool camera_control = false;//true for camera, false for mag
	uint8_t current_loop_state = 0;
	//

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

	uint32_t on9lastSent = 0, on9lastMain = 0;
	vector<Corner> slave_corner;

	//initialize
	servoPIDLoop.setkP(loop_servo_pd[0]);
	servoPIDLoop.setkD(loop_servo_pd[1]);
	servoPIDx.setkP(x_servo_pd[0]);
	servoPIDx.setkD(x_servo_pd[1]);
	servoPIDy.setkP(y_servo_pd[0]);
	servoPIDy.setkD(y_servo_pd[1]);
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
	uint32_t dTime = 0;
	while (1) {
		if (System::Time() != lastTime) {

			lastTime = System::Time();

//			 bt send motor speed
			if (lastTime - on9lastSent >= 100) {
				on9lastSent = lastTime;
				uart0.Send_float(DualCar_UART::FLOAT::f10, left_motorPID.getcurrentVelocity());
				uart0.Send_float(DualCar_UART::FLOAT::f11, right_motorPID.getcurrentVelocity());
				uart0.Send_float(DualCar_UART::FLOAT::f12, right_motorPID.getdTime());
//				uart0.Send_float(DualCar_UART::FLOAT::f12, mag.GetXLinear());
//				uart0.Send_float(DualCar_UART::FLOAT::f13, mag.GetYLinear());
			}

			if (USsent) {
				// moved away from ultrasonic sensor trig
				// feeling bad to put so many stuff in an interrupt

				buzz.SetNote(1040);
				buzz.SetBeep(true);

				approaching = false;
				isFirst = false;
				firstArrived = false;
				secondArrived = false;
				USsent = false;
				approachTime = lastTime;
			}

			uart0.RunEveryMS();
			led0.SetEnable(!approaching);
			led1.SetEnable(!isFirst);
			led2.SetEnable(!firstArrived);
			led3.SetEnable(!secondArrived);
			mag.TakeSample();
			batterySum += batteryMeter.GetVoltage();
			batteryCount++;

			if (lastTime - on9lastMain >= cycle) {
				const Byte* camBuffer = camera.LockBuffer();
				camera.UnlockBuffer();
				mag.Update();
				if (menu.get_mode() != DualCar_Menu::Page::kStart){
					lcd.SetRegion(Lcd::Rect(100,100,10,10));
					if (mag.isLoop()){
						lcd.FillColor(0xFF00);
					}else if (mag.unlikelyCrossRoad()){
						lcd.FillColor(0x00FF);
					}else {
						lcd.FillColor(0x0000);
					}
				}
				dot_time++;
				on9lastMain = lastTime;
				if (cali) {
					mag.Calibrate();
				}
				if (mag.noMagField() && magState == kNormal && menu.get_mode() == DualCar_Menu::Page::kStart && menu.get_selected()) {
					left_motorPID.setDesiredVelocity(0);
					right_motorPID.setDesiredVelocity(0);
					menu.select_pressed();
				}

//				if (batteryVoltage < 7.3 && !(menu.get_mode() == DualCar_Menu::Page::kStart && menu.get_selected())){
//					buzz.SetBeep(lastTime % 100 < 50);
//					lcd.SetRegion(Lcd::Rect(0,0,100,100));
//					lcd.FillColor(0xF100);
//				}

				//for alignment
				if (magState == kNormal && approaching) {
					magState = kLeave;
				} else if (magState == kLeave && mag.GetMag(Mag::magPos::x_left) < 15 && mag.GetMag(Mag::magPos::x_right) < mag.GetEMin()) {
					if (isFirst){
						magState = kStop;
						speed = 0;
					} else {
						magState = kSide;
						secondArrived = true;
						uart0.Send_bool(DualCar_UART::BOOLEAN::b2, true);
					}
				} else if (magState == kStop && secondArrived){
					magState = kSide;
					speed = alignSpeed;
					approachTime = lastTime;
				} else if (magState == kAlign && abs(mag.GetYDiff()) < 4 && abs(angle-middleServo) < 50){
					magState = kSide;
					approachTime = lastTime;
				} else if (magState == kSide && !approaching) {
					magState = kNormal;
					if (isFirst || firstArrived || secondArrived){
						isFirst = false;
						firstArrived = false;
						secondArrived = false;
						if (isFirst) {
							lcd.SetRegion(Lcd::Rect(0,0,100,50));
							lcd.FillColor(0xFF00);
						}
						if (firstArrived) {
							lcd.SetRegion(Lcd::Rect(0,50,100,50));
							lcd.FillColor(0xFF00);
						}
						if (secondArrived) {
							lcd.SetRegion(Lcd::Rect(0,100,100,50));
							lcd.FillColor(0xFF00);
						}
					}
//					speed = highSpeed;
					speed = 0;
					lastServo = -100;
				}

				if (!rubbishJoseph){
					servoPIDLoop.setkP(loop_servo_pd[0]);
					servoPIDLoop.setkD(loop_servo_pd[1]);
					servoPIDx.setkP(x_servo_pd[0]);
					servoPIDx.setkD(x_servo_pd[1]);
					servoPIDy.setkP(y_servo_pd[0]);
					servoPIDy.setkD(y_servo_pd[1]);
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
				}
				master_edge = left_edge.check_edge(camBuffer, 30, 60);
				vector<Corner> master_corner;
				master_corner = check_corner(camBuffer, 30, 60, master_edge);
				slave_corner = m_master_bluetooth.get_slave_corner();

				//alignment
				if(((master_corner.size()>1 || slave_corner.size()>1))&&(master_corner.size()!=0)&& (slave_corner.size()!=0) && mag.unlikelyCrossRoad() && !start_count_corner){
					dot_time = 0;
					start_count_corner = true;
				}

				if(start_count_corner){
					if(dot_time==10){
						start_count_corner = false;
						dot_time = 0;
						if(accumulate_corner>5){//original 4
							is_dot_line = true;
							buzz.SetBeep(true);
							if(!approaching && (lastTime - approachTime >= 15000 || approachTime == 0)){
								approaching = true;
								if (!firstArrived){
									isFirst = true;
									firstArrived = true;
									uart0.Send_bool(DualCar_UART::BOOLEAN::b1, true);
								}
							}
						}
						else{
							is_dot_line = false;
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


				//loop ver2
				float master_slope = 0;
				float slave_slope = 0;
				int s_edge_xmid = m_master_bluetooth.get_edge_xmid();
				int m_edge_xmid = 0;
				int mid_xmid = 40;
				if(master_edge.size()>0)
					m_edge_xmid = master_edge[master_edge.size()/2].first;
				int slave_edge_size = m_master_bluetooth.get_edge_size();
				master_slope = find_slope(master_edge);
				slave_slope = m_master_bluetooth.get_m_slope();
				if(mag.isLoop() && !in_loop){
					in_loop = true;
				}

				if(slave_edge_size<2){
					s_edge_xmid = 80;
				}

				current_loop_state = loop_control(current_loop_state, in_loop, &mag, left_loop, camera_control, camera_angle, master_edge.size(), slave_edge_size, m_edge_xmid, s_edge_xmid, master_corner.size(), slave_corner.size(),master_slope, slave_slope);

				if(!camera_control){
					if (cali) {
						angle = 0;
					} else if (magState == kNormal || magState == kLoop || magState == kExitLoop ||  magState == kLessTurn) {
//						angle = servoPIDAlignCurve.getPID(mag.GetEMin(0)*mag.GetMulti(0), mag.GetMag(0));
						float target = 0.0;
						if (magState == kLoop){
							if (right_loop){
								target = -0.005;
							} else{
								target = 0.005;
							}
						}
						angle = 0.5*servoPIDx.getPID(target, mag.GetXLinear()) + 0.5*servoPIDy.getPID(target, mag.GetYLinear());
							lastServo = angle;
						if (magState == kLessTurn){
							angle *= 0.5;
						}
					} else if (magState == kLeave){
						angle = 150;
					} else if (magState == kStop){
						angle = -400;
					} else if (magState == kAlign){
						angle = servoPIDAlignCurve.getPID(40, mag.GetMag(Mag::magPos::x_right));
					} else if (magState == kSide){
						if (mag.GetYSum() > 15){
							angle = 100 + 4 * servoPIDAlignCurve.getPID(0, mag.GetMag(Mag::magPos::x_left));
							buzz.SetNote(100);
							buzz.SetBeep(true);
						} else{
							angle = servoPIDAlignCurve.getPID(40, mag.GetMag(Mag::magPos::x_right));
							buzz.SetBeep(false);
							if (mag.GetMag(Mag::magPos::x_right) < 25){
								angle *= 1.5;
								buzz.SetNote(300);
								buzz.SetBeep(true);
							}
						}
					}
				} else{
					angle = camera_angle;
				}
				angle = 0.8*angle + 0.2*lastServo;
				angle += middleServo;
				angle = max(rightServo, min(leftServo, angle));
				servo.SetDegree(angle);

				if (angle > middleServo) {
					float angleRatio = 0.0013 * (angle - middleServo);
					float differential = angleRatio / (2 - angleRatio);
					left_motorPID.setDesiredVelocity(speed * (1 - differential));
					right_motorPID.setDesiredVelocity(speed * (1 + differential));
				} else {
					float angleRatio = 0.0013 * (middleServo - angle);
					float differential = angleRatio / (2 - angleRatio);
					left_motorPID.setDesiredVelocity(speed * (1 + differential));
					right_motorPID.setDesiredVelocity(speed * (1 - differential));
				}

				if (menu.get_mode() != DualCar_Menu::Page::kStart) {
					encoderL.Update();
					encoderR.Update();
					encoderLval = encoderL.GetCount();
					encoderRval = -encoderR.GetCount();
				}

				batteryVoltage = batterySum/batteryCount;
				if (menu.get_mode() == DualCar_Menu::Page::kStart){
					if (menu.get_selected()) {
						voltR = right_motorPID.getPID(cycle);
						voltL = left_motorPID.getPID(cycle);
						powerR = voltR/batteryVoltage*1000;
						powerL = voltL/batteryVoltage*1000;
						if (powerR > 0) {
							right_motor.SetClockwise(forwardR);
							right_motor.SetPower(min(powerR,1000));
						} else {
							right_motor.SetClockwise(!forwardR);
							right_motor.SetPower(min(-powerR,1000));
						}
						if (powerL > 0) {
							left_motor.SetClockwise(forwardL);
							left_motor.SetPower(min(powerL,1000));
						} else {
							left_motor.SetClockwise(!forwardL);
							left_motor.SetPower(min(-powerL,1000));
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
				} else{ //not start, printing show value
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
					Items item8("p", current_loop_state);
					Items item14("kind_l", left_loop);
					Items item15("S_es", slave_edge_size);

					Items item16("speed", speed);
					Items item17("r_en", encoderRval);
					Items item18("l_en", encoderLval);
					Items item19("lines", menu.get_line());
					Items item20("selected", menu.get_selected());
					Items item21("", dot_time);
					Items item22("Ultra", UltrasonicSensor.getDistance());


					Items item23("xL", mag.GetMag(Mag::magPos::x_left));
					Items item24("xR", mag.GetMag(Mag::magPos::x_right));
					Items item25("yL", mag.GetMag(Mag::magPos::y_left));
					Items item26("yR", mag.GetMag(Mag::magPos::y_right));
					Items item27("sum", mag.GetXSum()+mag.GetYSum());
					Items item28("magState", (int)magState);
					Items item29("l", mag.GetXLinear());
					Items item30("ac_cor", accumulate_corner);

					// show if its connected
					// 1: yes, 0: no
					Items item31("BTconn", uart0.isConnected());

					// show the run time of the client in ms
					Items item32("BTtime", uart0.receivedElpasedTime);

					// show the receive buffer of this
					Items item33("RxSize", uart0.RxBuffer.size());

					// show the send buffer of this
					Items item34("SeSize", uart0.SendImmediate.size());

					Items item35("getst", uart0.getsth);
					Items item36("sdsth", uart0.sendsth);
					Items item37("volt", batteryMeter.GetVoltage());


					mode0.add_items(&item0);
					mode0.add_items(&item1);
					mode0.add_items(&item2);
					mode0.add_items(&item3);
					mode0.add_items(&item4);
					mode0.add_items(&item5);

					mode1.add_items(&item6);
					mode1.add_items(&item1);
					mode1.add_items(&item8);
					mode1.add_items(&item14);
					mode1.add_items(&item15);

					mode2.add_items(&item16);
					mode2.add_items(&item17);
					mode2.add_items(&item18);
					mode2.add_items(&item19);
					mode2.add_items(&item20);
					mode2.add_items(&item21);
					mode2.add_items(&item22);

					mode3.add_items(&item23);
					mode3.add_items(&item24);
					mode3.add_items(&item25);
					mode3.add_items(&item26);
					mode3.add_items(&item27);
					mode3.add_items(&item28);
					mode3.add_items(&item29);
					mode3.add_items(&item30);

					mode4.add_items(&item31);
					mode4.add_items(&item32);
					mode4.add_items(&item33);
					mode4.add_items(&item34);
					mode4.add_items(&item35);
					mode4.add_items(&item36);
					mode4.add_items(&item37);
				}

				menu.add_mode(&mode0);
				menu.add_mode(&mode1);
				menu.add_mode(&mode2);
				menu.add_mode(&mode3);
				menu.add_mode(&mode4);
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
