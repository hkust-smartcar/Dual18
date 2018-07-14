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
#include "MenuV2.h"

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
	kEnter,
	kLoop,
	kExitLoop
} carState;
carState magState = kNormal;

static const uint8_t cycle = 12;
static float loopSpeed = 9.5, highSpeed = 9.5, alignSpeed = 9;
static float speed = highSpeed;
static float l1 = 100,l2 = 100,r1 = 100,r2 = 100;

int loop_control(int state, bool &is_loop, Mag* magnetic, bool &left_loop, int &camera_control, float &camera_angle, int master_edge_size, int slave_edge_size, int m_edge_xmid, int s_edge_xmid, int master_corner_size, int slave_corner_size, float master_slope, float slave_slope){
	static bool left;
	left_loop = left;
	static bool cameraReady = false, magReady = false;
	static float lastY = 0, rateY = 0, currY = 0, prevRateY = 0;
	switch(state){
	case 0:
		if(is_loop){
			state = 1;
			magState = kEnter;
		}
		break;
	case 1:
		if(master_slope < (-slave_slope))
			left = true;
		else
			left = false;
		state = 2;
		l1 = master_slope;
		r1 = slave_slope;
		break;
	case 2:
		currY = magnetic->GetYSum();
		prevRateY = rateY;
		rateY = currY - lastY;
		rateY = 0.7*rateY + 0.3*prevRateY;
		lastY = currY;
		if (rateY < 0){
			state = 3;
			l2 = master_slope;
			r2 = slave_slope;
		}
		break;
	case 3:
		if(left){
			camera_control = true;
			float difference = (40 - m_edge_xmid)/40.0;
			if(difference<0.5){
				camera_angle = difference*110;
			}
			else{
				camera_angle = difference*210;
			}
			if(slave_edge_size<4){
				state = 4;
				cameraReady = false;
				magReady = false;
			}
		}else{
			camera_control = true;
			float difference = (40 - s_edge_xmid)/40.0;
			if(difference>-0.5){
				camera_angle = difference*110;
			}
			else{
				camera_angle = difference*210;
			}
			if(master_edge_size<4){
				state = 4;
				cameraReady = false;
				magReady = false;
			}
		}
		break;
	case 4:
		if(left){
			float difference = (40 - m_edge_xmid)/40.0;
			if(difference<0.5){
				camera_angle = difference*150;
			}
			else{
				camera_angle = difference*250;
			}
			if((slave_edge_size>5)||(slave_corner_size==1)){
				cameraReady = true;
			}
			if (magnetic->GetXSum() < 100){
				magReady = true;
			}
			if(cameraReady && magReady){
				state = 5;
				camera_control = false;
				magState = carState::kLoop;
			}
		}
		else{
			float difference = (40 - s_edge_xmid)/40.0;
			if(difference>-0.5){
				camera_angle = difference*150;
			}
			else{
				camera_angle = difference*250;
			}
			if((master_edge_size>5)||(master_corner_size==1)){
				state = 5;
				camera_control = false;
				magState = carState::kLoop;
			}
		}
		break;
	case 5:
		if(magnetic->outLoop()){
			state = 6;
			magState = carState::kExitLoop;
			lastY = magnetic->GetYSum();
		}
		break;
	case 6:
		currY = magnetic->GetYSum();
		prevRateY = rateY;
		rateY = currY - lastY;
		rateY = 0.7*rateY + 0.3*prevRateY;
		lastY = currY;
		if (rateY < 0 && magnetic->GetYSum() < 110){
			state = 7;
			camera_control = true;
			magState = carState::kNormal;
		}
		break;
	case 7:
		if(left){
			if(slave_edge_size<2){
				s_edge_xmid = 0;
			}
			float difference = (40 - s_edge_xmid)/40.0;
			if(((difference<0.5)&&(difference>0))||((difference>-0.5)&&(difference<0))){
				camera_angle = difference*150;
			}
			else{
				camera_angle = difference*250;
			}
			if (magnetic->GetXSum() < 105 && magnetic->GetYSum() < 50){
				state = 0;
				camera_control = false;
				is_loop = false;
			}
		}
		else{
			if(master_edge_size<2){
				m_edge_xmid = 80;
			}
			float difference = (40 - m_edge_xmid)/40.0;
			if(((difference<0.5)&&(difference>0))||((difference>-0.5)&&(difference<0))){
				camera_angle = difference*150;
			}
			else{
				camera_angle = difference*250;
			}
			if (magnetic->GetXSum() < 105 && magnetic->GetYSum() < 50){
				state = 0;
				camera_control = false;
				is_loop = false;
			}
		}
		break;
	default:
		break;
	}
	return state;
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
	PID servoPIDx(0, 0);
	PID servoPIDy(0, 0);
	PID servoPIDAlignCurve(0, 0);
	PID cameraPID(0,0);
	PID left_motorPID(0, 0, 0, &encoderL, false);
	PID right_motorPID(0, 0, 0, &encoderR, true);

	Edge left_edge(false);

	//pid value

	float left_motor_pid[3],right_motor_pid[3],x_servo_pd[2],y_servo_pd[2],align_servo_pd[2];
	bool forwardL, forwardR;
	uint16_t middleServo, leftServo, rightServo;

	if (board.isCar1()) {
	    left_motor_pid[0] = 0.55;
	    left_motor_pid[1] = 0.012;
	    left_motor_pid[2] = 0.03;

	    right_motor_pid[0] = 0.55;
	    right_motor_pid[1] = 0.012;
	    right_motor_pid[2] = 0.03;

	    x_servo_pd[0] = 9800;
	    x_servo_pd[1] = 740000;

	    y_servo_pd[0] = 3.85;
	    y_servo_pd[1] = 228.73725;

		align_servo_pd[0] = 5.8;
		align_servo_pd[1] = 750;

		forwardL = true;
		forwardR = true;

		middleServo = 1045;
		leftServo = 1340;
		rightServo = 705;

		mag.InitMag(1);
	} else {
	    left_motor_pid[0] = 0.72;
	    left_motor_pid[1] = 0.025;
	    left_motor_pid[2] = 0.0001;

	    right_motor_pid[0] = 0.72;
	    right_motor_pid[1] = 0.025;
	    right_motor_pid[2] = 0.0001;

	    x_servo_pd[0] = 12000;
	    x_servo_pd[1] = 1100000;

	    y_servo_pd[0] = 4.4;
	    y_servo_pd[1] = 190;

		align_servo_pd[0] = 5.8;
		align_servo_pd[1] = 750;//car1 value

		forwardL = false;
		forwardR = true;

		middleServo = 840;
		leftServo = 1130;
		rightServo = 560;

		mag.InitMag(2);
	}

	uint32_t lastTime = 0, approachTime = 0;
	bool approaching = false, isFirst = false, firstArrived = false, secondArrived = false, USsent = false;
	bool cali = false;

	float angle = middleServo, angleX = 0, angleY = 0;
	float camera_angle = middleServo;
	float lastServo = 0;
	uint8_t leaveCount = 0;

	uint8_t cycleTime = 0;
	float encoderLval, encoderRval;
	float voltL, voltR;
	int32_t powerL, powerR;
	float batterySum = 0;
	uint16_t batteryCount = 0;


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

	//dotted lines
	uint32_t accumulate_corner = 0;
	uint8_t dot_time = 0;
	bool start_count_corner = false;

	bool rubbishJoseph = false;//two car

	//for loop ver2
	bool left_loop = false;
	bool in_loop = false;
	int camera_control = false;
	int current_loop_state = 0;
	//

	//menu v2
	DualCarMenu menuV2(&lcd, &writer, Width, Height);
	DualCarMenu::SubMenu* current_page = &menuV2.home_page;

	int temp = 5230;
	float tempf = 23.24;
	menuV2.AddItem("start", &(menuV2.home_page), true);
	menuV2.AddItem("OpenMotor", menuV2.home_page.submenu_items[0].next_page, true);
	menuV2.AddItem("CloseMotor", menuV2.home_page.submenu_items[0].next_page, true);

	menuV2.AddItem("camera", &(menuV2.home_page), true);
	menuV2.AddItem("image", menuV2.home_page.submenu_items[1].next_page, true);

	menuV2.AddItem("magnetic", &(menuV2.home_page), true);
	int mag_xL = mag.GetMag(Mag::magPos::x_left);
	int mag_xR = mag.GetMag(Mag::magPos::x_right);
	int mag_yL = mag.GetMag(Mag::magPos::y_left);
	int mag_yR = mag.GetMag(Mag::magPos::y_right);
	int mag_xSum = mag.GetXSum();
	int mag_ySum = mag.GetYSum();
	int* pmag_xL = &mag_xL;
	int* pmag_xR = &mag_xR;
	int* pmag_yL = &mag_yL;
	int* pmag_yR = &mag_yR;
	int* pmag_xSum = &mag_xSum;
	int* pmag_ySum = &mag_ySum;
	menuV2.AddItem("XL", pmag_xL, menuV2.home_page.submenu_items[2].next_page, false);
	menuV2.AddItem("XR", pmag_xR, menuV2.home_page.submenu_items[2].next_page, false);
	menuV2.AddItem("YL", pmag_yL, menuV2.home_page.submenu_items[2].next_page, false);
	menuV2.AddItem("YR", pmag_yR, menuV2.home_page.submenu_items[2].next_page, false);
	menuV2.AddItem("SX", pmag_xSum, menuV2.home_page.submenu_items[2].next_page, false);
	menuV2.AddItem("SY", pmag_ySum, menuV2.home_page.submenu_items[2].next_page, false);
	menuV2.AddItem("AX", &angleX, menuV2.home_page.submenu_items[2].next_page, false);
	menuV2.AddItem("AY", &angleY, menuV2.home_page.submenu_items[2].next_page, false);
//	menuV2.AddItem("temp", &temp, menuV2.home_page.submenu_items[2].next_page, true);
//	menuV2.AddItem("tempf", &tempf, menuV2.home_page.submenu_items[2].next_page, true);

	menuV2.AddItem("loop", &(menuV2.home_page), true);
	menuV2.AddItem("state", &current_loop_state, menuV2.home_page.submenu_items[3].next_page, false);
	menuV2.AddItem("cam_con", &camera_control, menuV2.home_page.submenu_items[3].next_page, false);
	menuV2.AddItem("L1", &l1, menuV2.home_page.submenu_items[3].next_page, false);
	menuV2.AddItem("L2", &l2, menuV2.home_page.submenu_items[3].next_page, false);
	menuV2.AddItem("R1", &r1, menuV2.home_page.submenu_items[3].next_page, false);
	menuV2.AddItem("R2", &r2, menuV2.home_page.submenu_items[3].next_page, false);

	int intmagState = (int)magState;
	int* pmagState = &intmagState;
	menuV2.AddItem("MagSt", pmagState, &(menuV2.home_page), false);

	menuV2.AddItem("other", &(menuV2.home_page), true);
	float distance = UltrasonicSensor.getDistance();
	menuV2.AddItem("Dist", &(distance), menuV2.home_page.submenu_items[5].next_page, false);
	menuV2.AddItem("Volt", &(batteryVoltage), menuV2.home_page.submenu_items[5].next_page, false);
	menuV2.AddItem("EncL", &(encoderLval), menuV2.home_page.submenu_items[5].next_page, false);
	menuV2.AddItem("EncR", &(encoderRval), menuV2.home_page.submenu_items[5].next_page, false);
	int mpu_data = 0;
	int* pmpu_data = &mpu_data;
	menuV2.AddItem("mpu", pmpu_data, menuV2.home_page.submenu_items[5].next_page, false);

	Joystick js(myConfig::GetJoystickConfig(Joystick::Listener([&]
	(const uint8_t id, const Joystick::State state) {
		menuV2.SetJoystickState(state);
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
	servoPIDx.setkP(x_servo_pd[0]);
	servoPIDx.setkD(x_servo_pd[1]);
	servoPIDy.setkP(y_servo_pd[0]);
	servoPIDy.setkD(y_servo_pd[1]);
	servoPIDAlignCurve.setkP(align_servo_pd[0]);
	servoPIDAlignCurve.setkD(align_servo_pd[1]);
	left_motorPID.setkP(left_motor_pid[0]);
	left_motorPID.setkI(left_motor_pid[1]);
	left_motorPID.setkD(left_motor_pid[2]);
	right_motorPID.setkP(right_motor_pid[0]);
	right_motorPID.setkI(right_motor_pid[1]);
	right_motorPID.setkD(right_motor_pid[2]);
	uint32_t dTime = 0;


	bool bumpy_road = false;

	while (1) {
		if (System::Time() != lastTime) {

			lastTime = System::Time();

//			 bt send motor speed
			if (lastTime - on9lastSent >= 50) {
				on9lastSent = lastTime;
				uart0.Send_float(DualCar_UART::FLOAT::f10, left_motorPID.getcurrentVelocity());
				uart0.Send_float(DualCar_UART::FLOAT::f11, right_motorPID.getcurrentVelocity());
				uart0.Send_float(DualCar_UART::FLOAT::f12, mag.GetXLinear());
				uart0.Send_float(DualCar_UART::FLOAT::f13, mag.GetYLinear());
				uart0.Send_float(DualCar_UART::FLOAT::f21, servoPIDx.getpTerm());
				uart0.Send_float(DualCar_UART::FLOAT::f22, servoPIDx.getdTerm());
				uart0.Send_float(DualCar_UART::FLOAT::f23, servoPIDy.getpTerm());
				uart0.Send_float(DualCar_UART::FLOAT::f24, servoPIDy.getdTerm());
			}

			if (USsent) {
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
//				if (menu.get_mode() != DualCar_Menu::Page::kStart){
//					lcd.SetRegion(Lcd::Rect(100,100,10,10));
//					if (mag.isLoop()){
//						lcd.FillColor(0xFF00);
//					}else if (mag.unlikelyCrossRoad()){
//						lcd.FillColor(0x00FF);
//					}else {
//						lcd.FillColor(0x0000);
//					}
//				}
				on9lastMain = lastTime;
				if (cali) {
					mag.Calibrate();
				}
				if (mag.noMagField() && current_page->identity == "OpenMotor") {
					speed = 0;
				}

				//for alignment
				if (magState == kNormal && approaching) {
					magState = kLeave;
					leaveCount = 0;
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
					speed = highSpeed;
				}

				if (!rubbishJoseph){
					servoPIDx.setkP(x_servo_pd[0]);
					servoPIDx.setkD(x_servo_pd[1]);
					servoPIDy.setkP(y_servo_pd[0]);
					servoPIDy.setkD(y_servo_pd[1]);
					servoPIDAlignCurve.setkP(align_servo_pd[0]);
					servoPIDAlignCurve.setkD(align_servo_pd[1]);
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
				if((mpu_data>3000)||(mpu_data<-3000)){
					bumpy_road = true;
					led0.SetEnable(0);
				}
				else{
					bumpy_road = false;
					led0.SetEnable(1);
				}

				//alignment
				if(((master_corner.size()>1 || slave_corner.size()>1))&&(master_corner.size()!=0)&& (slave_corner.size()!=0) && (!start_count_corner)&&(!bumpy_road)){
					dot_time = 0;
					start_count_corner = true;
				}


				if(start_count_corner){
					dot_time++;
					if(dot_time==10){
						if(accumulate_corner > 5){
							buzz.SetBeep(true);
							if(!approaching && !mag.isTwoLine() && mag.unlikelyCrossRoad() && (lastTime - approachTime >= 10000 || approachTime == 0)){
								approaching = true;
								if (!firstArrived){
									isFirst = true;
									firstArrived = true;
									uart0.Send_bool(DualCar_UART::BOOLEAN::b1, true);
								}
							}
						}
						accumulate_corner = 0;
						start_count_corner = false;
						dot_time = 0;
					}
					else{
						accumulate_corner += master_corner.size();
						accumulate_corner += slave_corner.size();
					}
				}
				//

				(*pmpu_data) = m_master_bluetooth.get_mpu_data();


				//loop ver2
				float master_slope = 0;
				float slave_slope = 0;
				int s_edge_xmid = m_master_bluetooth.get_edge_xmid();
				int m_edge_xmid = 0;
				if(master_edge.size()>0)
					m_edge_xmid = master_edge[master_edge.size()/2].first;
				int slave_edge_size = m_master_bluetooth.get_edge_size();
				master_slope = find_slope(master_edge);
				slave_slope = m_master_bluetooth.get_m_slope();

				if(mag.isLoop() && !in_loop && !bumpy_road){
					in_loop = true;
				}

				if (current_loop_state > 0 && current_loop_state < 6 && left_loop){
					buzz.SetNote(440);
					buzz.SetBeep(true);
				}

				if(slave_edge_size<2){
					s_edge_xmid = 80;
				}

				if (in_loop){
					current_loop_state = loop_control(current_loop_state, in_loop, &mag, left_loop, camera_control, camera_angle, master_edge.size(), slave_edge_size, m_edge_xmid, s_edge_xmid, master_corner.size(), slave_corner.size(),master_slope, slave_slope);
				}

				if(!camera_control){
					if (cali || mag.noMagField()) {
						angle = 0;
					} else if (magState == kNormal || magState == kLoop || magState == kExitLoop) {
//						angle = servoPIDAlignCurve.getPID(mag.GetEMin(0)*mag.GetMulti(0), mag.GetMag(0));
						float target = 0.0;
						if (magState == kLoop || magState ==kExitLoop){
							if (left_loop){
								target = 0.02;
							} else{
								target = -0.02;
							}
						}
						angleX = servoPIDx.getPID(target, mag.GetXLinear());
						angleY = servoPIDy.getPID(0, mag.GetYLinear());
						if (mag.isTwoLine() && magState == kNormal){
							angle = 0.25*angleX + angleY;
						} else if (mag.GetYSum() > 15 && mag.GetXSum() < 80 && ((angleX > 0) ^ (angleY > 0)) && magState == kNormal){
							angle = angleY;
						} else{
							angle = 0.5*angleX + 0.5*angleY;
							buzz.SetBeep(false);
						}
						if (magState == kExitLoop){
							buzz.SetNote(659);
							buzz.SetBeep(true);
						}
					} else if (magState == kEnter){
						angleY = servoPIDy.getPID(0, mag.GetYLinear());
						angle = angleY;
					} else if (magState == kLeave){
//						angle = 150;
						angle = max(300-leaveCount*20, 100);
						leaveCount++;
					} else if (magState == kStop){
						angle = -400;
					} else if (magState == kAlign){
						angle = -servoPIDAlignCurve.getPID(40, mag.GetMag(Mag::magPos::x_right));
					} else if (magState == kSide){
						if (mag.GetYSum() > 15){
							angle = 100 - 5 * servoPIDAlignCurve.getPID(0, mag.GetMag(Mag::magPos::x_left));
							angleY = angle;
							buzz.SetNote(100);
							buzz.SetBeep(true);
						} else{
							angle = -servoPIDAlignCurve.getPID(40, mag.GetMag(Mag::magPos::x_right));
							buzz.SetBeep(false);
							if (mag.GetMag(Mag::magPos::x_right) < 25){
//								angle *= 1.5;
								angle *= 1 + ((25-mag.GetMag(Mag::magPos::x_right)) * 0.005);
								buzz.SetNote(300);
								buzz.SetBeep(true);
							}
							angleX = angle;
						}
					}
				} else{
					angle = camera_angle;
					buzz.SetBeep(false);
				}
				angle = 0.75*angle + 0.25*lastServo;
				angle = max(rightServo-middleServo, min(leftServo-middleServo, angle));
				lastServo = angle;
				angle += middleServo;
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

				/////print menu
				if(left_loop){
					led0.SetEnable(0);
					led1.SetEnable(0);
					led2.SetEnable(0);
					led3.SetEnable(0);
				}
				*pmagState = (int)magState;
				if (cali){
					mag_xL = mag.GetRaw(Mag::magPos::x_left);
					mag_xR = mag.GetRaw(Mag::magPos::x_right);
					mag_yL = mag.GetRaw(Mag::magPos::y_left);
					mag_yR = mag.GetRaw(Mag::magPos::y_right);
				} else{
					mag_xL = mag.GetMag(Mag::magPos::x_left);
					mag_xR = mag.GetMag(Mag::magPos::x_right);
					mag_yL = mag.GetMag(Mag::magPos::y_left);
					mag_yR = mag.GetMag(Mag::magPos::y_right);
				}
				mag_xSum = mag.GetXSum();
				mag_ySum = mag.GetYSum();
				distance = UltrasonicSensor.getDistance();
				menuV2.SetCamBuffer(camBuffer);
				menuV2.SetEdge(master_edge);
				current_page = menuV2.PrintSubMenu(current_page);
				batteryVoltage = batterySum/batteryCount;
				if(current_page->identity == "OpenMotor"){
					voltR = right_motorPID.getPID();
					voltL = left_motorPID.getPID();
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
				}
				else{
					encoderL.Update();
					encoderR.Update();
					encoderLval = encoderL.GetCount();
					encoderRval = -encoderR.GetCount();
					left_motorPID.setDesiredVelocity(0);
					right_motorPID.setDesiredVelocity(0);
					left_motor.SetPower(0);
					right_motor.SetPower(0);
				}
				/////

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
