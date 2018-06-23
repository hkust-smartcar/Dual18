/*
 * master_main.cpp
 *
 *  Created on: Jun 21, 2018
 *      Author: morristseng
 */

#define Master
#define car1

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
	buzz.SetNote(523);
	buzz.SetBeep(batteryVoltage < 7);

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
	PID servoPIDAlignCurve(-10, 0);
    PID servoPIDAlignStraight(-4,10);
	PID left_motorPID(0, 0, 0, &encoderL, false);
	PID right_motorPID(0, 0, 0, &encoderR, true);

	//pid value
#ifdef car1
	float left_motor_pid[3] = { 0.16, 0.001, 0.00004 };
	float right_motor_pid[3] = { 0.16, 0.001, 0.00004 };
	float straight_servo_pd[2] = { 1600, 2000 };
	float curve_servo_pd[2] = { 1600, 2000 };
#endif
#ifdef car2
	float left_motor_pid[3] = { 0.16, 0.001, 0.00004};
	float right_motor_pid[3] = { 0.16, 0.001, 0.00004 };
	float straight_servo_pd[2] = { 1600, 2000 };
	float curve_servo_pd[2] = { 1600, 2000 };
#endif

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

	const uint16_t middleServo = 850, leftServo = 1150, rightServo = 550;
	float angle = middleServo;
	float lastServo = 0;

	uint8_t cycleTime = 0;
	const uint8_t cycle = 10;
	float loopSpeed = 4 * cycle, highSpeed = 6 * cycle, alignSpeed = 6 * cycle;
	float speed = highSpeed;
//	volatile
	float encoderLval, encoderRval;
	float voltL, voltR;
	int32_t powerL, powerR;

	DualCar_UART uart0(1); // << BT related

	bool turn_on_motor = false;

	// below sync data to the computer side
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

	uart0.parseValues();
	//

	//joystick value
	DualCar_Menu menu;
	Mode mode0(0);
	Mode mode1(1);
	Mode mode2(2);
	Mode mode3(3);
	Mode ClearMode(4);
	//

	//printing change value
	Items item5("sr_kp", &straight_servo_pd[0], true);
	Items item6("sr_kd", &straight_servo_pd[1], true);
	Items item7("cr_kp", &curve_servo_pd[0], true);
	Items item8("cr_kd", &curve_servo_pd[1], true);
	item5.set_increment(100);
	item6.set_increment(100);
	item7.set_increment(100);
	item8.set_increment(100);

	Items item13("ml_kp", &left_motor_pid[0], true);
	Items item14("ml_ki", &left_motor_pid[1], true);
	Items item15("ml_kd", &left_motor_pid[2], true);
	item13.set_increment(0.1);
	item14.set_increment(0.1);
	item15.set_increment(0.1);

	Items item16("mr_kp", &right_motor_pid[0], true);
	Items item17("mr_ki", &right_motor_pid[1], true);
	Items item18("mr_kd", &right_motor_pid[2], true);
	item16.set_increment(0.1);
	item17.set_increment(0.1);
	item18.set_increment(0.1);
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
	vector<pair<int, int>> temp;
	vector<pair<int, int>> midline;
	M_Bluetooth m_master_bluetooth;

	vector<pair<int, int>> m_slave_vector;
	vector<pair<int, int>> m_master_vector;

	int32_t on9lastSent = 0;
	int32_t on9lastMain = 0;
	float x = 0;

	while (1) {
		if (System::Time() != lastTime) {

			lastTime = System::Time();

			// bt send motor speed
			if (lastTime - on9lastSent > 50) {
				on9lastSent = lastTime;
				uart0.Send_float(DualCar_UART::FLOAT::f10, left_motorPID.getcurrentVelocity());
				uart0.Send_float(DualCar_UART::FLOAT::f11, right_motorPID.getcurrentVelocity());
			}

			uart0.RunEveryMS();
			mag.TakeSample();
			if (lastTime - on9lastMain >= cycle) {
				x += 0.02;
				on9lastMain = lastTime;
				const Byte* camBuffer = camera.LockBuffer();
				camera.UnlockBuffer();
				mag.Update();
				if (cali) {
					mag.Calibrate();
				}
//				if (turn_on_motor && mag.noMagField()) {
//					turn_on_motor = false;
//					left_motorPID.setDesiredVelocity(0);
//					right_motorPID.setDesiredVelocity(0);
//				}
				//for alignment
				if (state == normal && approaching) {
					state = leave;
					speed = alignSpeed;
				} else if (state == leave && mag.SmallerThanE(0, 0.5) && mag.SmallerThanE(1, 1.2)) {
					state = align;
				} else if (state == align && mag.Difference(0, 1) < 10 && abs((int) angle - middleServo) < 50) {
					state = side;
					approachTime = System::Time();
				} else if (state == side && !approaching) {
					state = back;
					lastServo = -300;
				} else if (state == back && !mag.SmallerThanE(0, 1)) {
					state = normal;
					speed = 0;
					//	speed = highSpeed;
				}

				double master_slope;
				double slave_slope;
				double midline_slope;

				servoPIDStraight.setkP(straight_servo_pd[0]);
				servoPIDStraight.setkD(straight_servo_pd[1]);
				servoPIDCurve.setkP(curve_servo_pd[0]);
				servoPIDCurve.setkD(curve_servo_pd[1]);
				left_motorPID.setkP(left_motor_pid[0]);
				left_motorPID.setkI(left_motor_pid[1]);
				left_motorPID.setkD(left_motor_pid[2]);
				right_motorPID.setkP(right_motor_pid[0]);
				right_motorPID.setkI(right_motor_pid[1]);
				right_motorPID.setkD(right_motor_pid[2]);

				temp = m_master_bluetooth.get_m_edge();
				bool left_fail = true;
				bool right_fail = true;
				vector<Corner> master_corner;
				vector<Corner> slave_corner;
				master_corner = check_corner(camBuffer, 20, 60, true);
				slave_corner = m_master_bluetooth.get_slave_corner();

				left_fail = check_left_edge(20, 60, camBuffer, m_master_vector);
				right_fail = m_master_bluetooth.get_fail_on_turn();
				master_slope = find_slope(m_master_vector);
				slave_slope = find_slope(temp);

				find_midline(m_master_vector, temp, midline);

				midline_slope = find_slope(midline);

				if (cali) {
					angle = middleServo;
				} else if (state == normal) {
					if (mag.SmallerThanMin(0, 2.5) || mag.SmallerThanMin(1, 2.5)){
						angle = lastServo * 1.3;
					} else {
						angle = servoPIDCurve.getPID(0.0, mag.GetLinear(0));
						lastServo = angle;
					}
				} else if (state == leave){
					angle = servoPIDAlignCurve.getPID(mag.GetEMin(0), mag.GetMag(0));
//					angle = servoPIDCurve.getPID(0.08,frontLinear);
				} else if (state == align) {
					angle = servoPIDAlignCurve.getPID(mag.GetEMin(1), mag.GetMag(1));
//					angle = servoPIDCurve.getPID(0.08,frontLinear);
					if (!mag.SmallerThanMin(0, 3) && angle < 0) {
						angle = -angle;
					}
				} else if (state == side){
					angle = servoPIDAlignStraight.getPID(mag.GetEMin(1), mag.GetMag(1));
//					angle = servoPIDStraight.getPID(0.08,frontLinear);
				} else if (state == back){
					angle = servoPIDAlignCurve.getPID(mag.GetEMax(0), mag.GetMag(0));
//					angle = servoPIDAlignStraight.getPID(equalMax, raw_front_left);
				}

				m_master_bluetooth.reset_m_edge();

				angle += middleServo;
				angle = max(rightServo, min(leftServo, angle));
				servo.SetDegree(angle);

				if (angle > middleServo) {
					float angleRatio = 0.0013 * (angle - middleServo);
					float differential = angleRatio / (2 - angleRatio);
//					left_motorPID.setDesiredVelocity(speed * (1 - differential));
//					right_motorPID.setDesiredVelocity(speed * (1 + differential));
//					left_motorPID.setDesiredVelocity(speed * sin(x));
//					right_motorPID.setDesiredVelocity(speed * sin(x));
					left_motorPID.setDesiredVelocity(speed);
					right_motorPID.setDesiredVelocity(speed);
				} else {
					float angleRatio = 0.0013 * (middleServo - angle);
					float differential = angleRatio / (2 - angleRatio);
//					left_motorPID.setDesiredVelocity(speed * (1 + differential));
//					right_motorPID.setDesiredVelocity(speed * (1 - differential));
//					left_motorPID.setDesiredVelocity(speed * sin(x));
//					right_motorPID.setDesiredVelocity(speed * sin(x));
					left_motorPID.setDesiredVelocity(speed);
					right_motorPID.setDesiredVelocity(speed);
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
							right_motor.SetClockwise(true);
							right_motor.SetPower(powerR);
						} else {
							right_motor.SetClockwise(false);
							right_motor.SetPower(-powerR);
						}
						if (powerL > 0) {
							left_motor.SetClockwise(false);
							left_motor.SetPower(powerL);
						} else {
							left_motor.SetClockwise(true);
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
					Items item0("Master");
					Items item1 ("B_Sl", midline_slope);
					Items item2("R_Sl", master_slope);
					Items item3("G_Sl", slave_slope);
					Items item4("Sv", servo.GetDegree());
					Items item25("corner", master_corner.size());


					Items item9("r_en", encoderRval);
					Items item10("l_en", encoderLval);
					Items item11("lines", menu.get_line());
					Items item12("selected", menu.get_selected());

					Items item19("speed", speed);
					Items item20("lines", menu.get_line());
					Items item21("selected", menu.get_selected());

					Items item22("left", mag.GetMag(0));
					Items item23("right", mag.GetMag(1));
					Items item24("time", lastTime % 100);
					Items item26("volt", batteryMeter.GetVoltage());


					mode0.add_items(&item0);
					mode0.add_items(&item1);
					mode0.add_items(&item2);
					mode0.add_items(&item3);
					mode0.add_items(&item4);
					mode0.add_items(&item25);

					mode1.add_items(&item5);
					mode1.add_items(&item6);
					mode1.add_items(&item7);
					mode1.add_items(&item8);
					mode1.add_items(&item9);
					mode1.add_items(&item10);
					mode1.add_items(&item11);
					mode1.add_items(&item12);

					mode2.add_items(&item13);
					mode2.add_items(&item14);
					mode2.add_items(&item15);
					mode2.add_items(&item16);
					mode2.add_items(&item17);
					mode2.add_items(&item18);
					mode2.add_items(&item19);
					mode2.add_items(&item20);
					mode2.add_items(&item21);

					mode3.add_items(&item22);
					mode3.add_items(&item23);
					mode3.add_items(&item24);
					mode3.add_items(&item26);
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
					for (int i = 0; i < m_master_vector.size(); i++) {
						lcd.SetRegion(
								Lcd::Rect(m_master_vector[i].first,
										m_master_vector[i].second, 2, 2));
						lcd.FillColor(Lcd::kRed);
					}
					for (int i = 0; i < temp.size(); i++) {
						lcd.SetRegion(
								Lcd::Rect(temp[i].first, temp[i].second, 2, 2));
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
				midline.clear();
				temp.clear();
				m_slave_vector.clear();
				m_master_vector.clear();

				cycleTime = System::Time() - lastTime;
			}
		}
	}
	return 0;
}

#endif