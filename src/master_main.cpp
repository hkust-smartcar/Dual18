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
	volatile float encoderLval, encoderRval;
	int32_t powerL, powerR;

	//DualCar_UART uart0; // << BT related
	bool turn_on_motor = false;

	//pid value
	float left_motor_pid[3] = { 10.00, 0.00, 0.00 };
	float right_motor_pid[3] = { 10.00, 0.00, 0.00 };
	float straight_servo_pd[2] = { 1600, 2000 };
	float curve_servo_pd[2] = { 1600, 2000 };
	//

	//joystick value
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
			line = 1;
			if(mode != 0){
				mode--;
			}
		}
		else if(state == Joystick::State::kRight){
			changed = true;
			select = false;
			select_time = 0;
			line = 1;
			if(mode != 3){
				mode++;
			}
		}
		else if (state == Joystick::State::kSelect) {
			if(mode == 1) {
				select_time++;
				if(select_time % 2 == 1)
				select = true;
				else
				select = false;
			}
			if(mode ==2) {
				select_time++;
				if(select_time%2==1)
				select = true;
				else
				select = false;
			}
			if(mode==3) {
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
			else if(mode==1) {
				if(!select) {
					if(line == 1)
					line=1;
					else
					line --;
				}
				else {
					switch(line%4) {
						case 1:
						straight_servo_pd[0]+= 100;
						break;
						case 2:
						straight_servo_pd[1]+= 100;
						break;
						case 3:
						curve_servo_pd[0]+= 100;
						break;
						case 0:
						curve_servo_pd[1]+= 100;
						break;
					}
				}
			}
			else if(mode==2) {
				if(!select) {
					if(line == 1)
					line=1;
					else
					line --;
				}
				else {
					switch(line%7) {
						case 1:
						left_motor_pid[0]+= 0.1;
						break;
						case 2:
						left_motor_pid[1]+= 0.1;
						break;
						case 3:
						left_motor_pid[2]+= 0.1;
						break;
						case 4:
						right_motor_pid[0]+= 0.1;
						break;
						case 5:
						right_motor_pid[1]+= 0.1;
						break;
						case 6:
						right_motor_pid[2]+= 0.1;
						break;
						case 7:
						speed += 10;
						break;
					}
				}
			}
		}
		else if (state == Joystick::State::kDown) {
			if(mode==0) {
			}
			else if(mode==1) {
				if(!select) {
					if(line >= 4)
					line=4;
					else
					line ++;
				}
				else {
					switch(line%4) {
						case 1:
						straight_servo_pd[0]-= 100;
						if(straight_servo_pd[0]<0)
						straight_servo_pd[0]=0;
						break;

						case 2:
						straight_servo_pd[1]-= 100;
						if(straight_servo_pd[1]<0)
						straight_servo_pd[1]=0;
						break;

						case 3:
						curve_servo_pd[0]-= 100;
						if(curve_servo_pd[0]<0)
						curve_servo_pd[0]=0;
						break;

						case 0:
						curve_servo_pd[1]-= 100;
						if(curve_servo_pd[1]<0)
						curve_servo_pd[1]=0;
						break;
					}
				}
			}
			else if(mode==2) {
				if(!select) {
					if(line >= 7)
					line=8;
					else
					line ++;
				}
				else {
					switch(line%7) {
						case 1:
						left_motor_pid[0]-= 0.1;
						if(left_motor_pid[0]<0)
						left_motor_pid[0] =0;
						break;
						case 2:
						left_motor_pid[1]-= 0.1;
						if(left_motor_pid[1]<0)
						left_motor_pid[1] =0;
						break;
						case 3:
						left_motor_pid[2]-= 0.1;
						if(left_motor_pid[2]<0)
						left_motor_pid[2] =0;
						break;
						case 4:
						right_motor_pid[0]-= 0.1;
						if(right_motor_pid[0]<0)
						right_motor_pid[0] =0;
						break;
						case 5:
						right_motor_pid[1]-= 0.1;
						if(right_motor_pid[1]<0)
						right_motor_pid[1] =0;
						break;
						case 6:
						right_motor_pid[2]-= 0.1;
						if(right_motor_pid[2]<0)
						right_motor_pid[2] =0;
						break;
						case 7:
						speed -= 10;
						if(speed<0)
						speed = 0;
						break;
					}
				}
			}
		}
	})));

	servo.SetDegree(middleServo);

	//bluetooth communicate
	vector<pair<int, int>> temp;
	vector<pair<int, int>> midline;
	M_Bluetooth m_master_bluetooth;

	vector<pair<int, int>> m_slave_vector;
	vector<pair<int, int>> m_master_vector;

	while (1) {
		if (System::Time() != lastTime) {

			lastTime = System::Time();

			//uart0.RunEveryMS();
			mag.TakeSample();
			if (lastTime % cycle == 0) {
				const Byte* camBuffer = camera.LockBuffer();
				camera.UnlockBuffer();
				mag.Update();
				if (cali) {
					mag.Calibrate();
//					oneLineMin = min(raw_front_left,
//							min(raw_front_right,
//									min(raw_mid_left,
//											min(raw_mid_right, oneLineMin))));
//					oneLineMax = max(raw_front_left,
//							max(raw_front_right,
//									max(raw_mid_left,
//											max(raw_mid_right, oneLineMax))));
//					topMax = max(raw_top_left, max(raw_top_right, topMax));
//					if (raw_front_left == raw_front_right) {
//						if (equalMin > 100
//								|| (equalMin > raw_front_left
//										&& equalMin - raw_front_left < 5)) {
//							equalMin = raw_front_left;
//
//						}
//						if (equalMax < 10
//								|| (equalMax < raw_front_left
//										&& raw_front_left - equalMax < 5)) {
//							equalMax = raw_front_left;
//						}
//					}
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
					left_motorPID.setDesiredVelocity(speed * (1 - differential));
					right_motorPID.setDesiredVelocity(speed * (1 + differential));
				} else {
					float angleRatio = 0.0013 * (middleServo - angle);
					float differential = angleRatio / (2 - angleRatio);
					left_motorPID.setDesiredVelocity(speed * (1 + differential));
					right_motorPID.setDesiredVelocity(speed * (1 - differential));
				}

				if (mode != 3) {
					encoderL.Update();
					encoderR.Update();
				}
				encoderLval = encoderL.GetCount();
				encoderRval = -encoderR.GetCount();

				if (turn_on_motor) {
					powerR = right_motorPID.getPID();
					powerL = left_motorPID.getPID();
					if (powerR > 0) {
						right_motor.SetClockwise(true);
						right_motor.SetPower(powerR);
					} else {
						right_motor.SetClockwise(false);
						right_motor.SetPower(-powerR);
					}
					if (powerL > 0) {
						left_motor.SetClockwise(true);
						left_motor.SetPower(powerL);
					} else {
						left_motor.SetClockwise(false);
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
					for (int i = 0; i < m_master_vector.size(); i++) {
						lcd.SetRegion(
								Lcd::Rect(m_master_vector[i].first,
										m_master_vector[i].second, 2, 2));
						lcd.FillColor(Lcd::kRed);
					}
					for (int i = 0; i < temp.size(); i++) {
						lcd.SetRegion(
								Lcd::Rect(temp[i].first, temp[i].second, 2, 2));
						lcd.FillColor(Lcd::kGreen);
					}
					lcd.SetRegion(
							Lcd::Rect(temp[0].first, temp[0].second, 2, 2));
					lcd.FillColor(Lcd::kPurple);

					for (int i = 0; i < midline.size(); i++) {
						lcd.SetRegion(
								Lcd::Rect(midline[i].first, midline[i].second,
										2, 2));
						lcd.FillColor(Lcd::kBlue);
					}
					lcd.SetRegion(Lcd::Rect(0, 60, 88, 15));
					sprintf(c, "B_Sl:%.2f", midline_slope);
					writer.WriteBuffer(c, 10);
					lcd.SetRegion(Lcd::Rect(0, 75, 88, 15));
					sprintf(c, "R_Sl:%.2f ", master_slope);
					writer.WriteBuffer(c, 10);
					lcd.SetRegion(Lcd::Rect(0, 90, 88, 15));
					sprintf(c, "G_Sl:%.2f ", slave_slope);
					writer.WriteBuffer(c, 10);
					for (int i = 0; i < 10; i++) {
						c[i] = ' ';
					}
					lcd.SetRegion(Lcd::Rect(0, 105, 88, 15));
					sprintf(c, "Sv:%d ", servo.GetDegree());\
					writer.WriteBuffer(c, 10);
				}

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
					sprintf(c, "sr_kp:%.3f", straight_servo_pd[0]);
					writer.WriteBuffer(c, 15);

					for (int i = 0; i < 15; i++) {
						c[i] = ' ';
					}
					lcd.SetRegion(Lcd::Rect(0, 15, 88, 15));
					sprintf(c, "sr_kd:%.3f", straight_servo_pd[1]);
					writer.WriteBuffer(c, 15);

					for (int i = 0; i < 15; i++) {
						c[i] = ' ';
					}
					lcd.SetRegion(Lcd::Rect(0, 30, 88, 15));
					sprintf(c, "sr_kp:%.3f", curve_servo_pd[0]);
					writer.WriteBuffer(c, 15);

					for (int i = 0; i < 15; i++) {
						c[i] = ' ';
					}
					lcd.SetRegion(Lcd::Rect(0, 45, 88, 15));
					sprintf(c, "sr_kd:%.3f", curve_servo_pd[1]);
					writer.WriteBuffer(c, 15);

					for (int i = 0; i < 15; i++) {
						c[i] = ' ';
					}
					lcd.SetRegion(Lcd::Rect(0, 90, 88, 15));
					sprintf(c, "r_en:%f", encoderRval);
					writer.WriteBuffer(c, 15);

					for (int i = 0; i < 15; i++) {
						c[i] = ' ';
					}
					lcd.SetRegion(Lcd::Rect(0, 105, 88, 15));
					sprintf(c, "l_en:%f", encoderLval);
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
					if (select) {
						lcd.SetRegion(Lcd::Rect(0, 135, 120, 15));
						sprintf(c, "select: true");
						writer.WriteBuffer(c, 15);
					} else {
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
					sprintf(c, "ml_kp:%.3f", left_motorPID.getkP());
					writer.WriteBuffer(c, 15);

					for (int i = 0; i < 15; i++) {
						c[i] = ' ';
					}
					lcd.SetRegion(Lcd::Rect(0, 15, 100, 15));
					sprintf(c, "ml_ki:%.3f", left_motorPID.getkI());
					writer.WriteBuffer(c, 15);

					for (int i = 0; i < 15; i++) {
						c[i] = ' ';
					}
					lcd.SetRegion(Lcd::Rect(0, 30, 100, 15));
					sprintf(c, "ml_kd:%.3f", left_motorPID.getkD());
					writer.WriteBuffer(c, 15);

					for (int i = 0; i < 15; i++) {
						c[i] = ' ';
					}
					lcd.SetRegion(Lcd::Rect(0, 45, 100, 15));
					sprintf(c, "mr_kp:%.3f", right_motorPID.getkP());
					writer.WriteBuffer(c, 15);

					for (int i = 0; i < 15; i++) {
						c[i] = ' ';
					}
					lcd.SetRegion(Lcd::Rect(0, 60, 100, 15));
					sprintf(c, "mr_ki:%.3f", right_motorPID.getkI());
					writer.WriteBuffer(c, 15);

					for (int i = 0; i < 15; i++) {
						c[i] = ' ';
					}
					lcd.SetRegion(Lcd::Rect(0, 75, 100, 15));
					sprintf(c, "mr_kd:%.3f", right_motorPID.getkD());
					writer.WriteBuffer(c, 15);

					for (int i = 0; i < 10; i++) {
						c[i] = ' ';
					}
					lcd.SetRegion(Lcd::Rect(0, 90, 88, 15));
					sprintf(c, "speed:%d ", speed);
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
					if (select) {
						lcd.SetRegion(Lcd::Rect(0, 135, 120, 15));
						sprintf(c, "select: true");
						writer.WriteBuffer(c, 15);
					} else {
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
