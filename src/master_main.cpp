/*
 * master_main.cpp
 *
 *  Created on: Jun 21, 2018
 *      Author: morristseng
 */

#define Master

#ifdef Master

#define temp_cam_fix

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
#include "DualCar_UART_Sim.h"
#include "mag_func.h"
#include "useful_functions.h"
#include "menu.h"
#include "DistanceModule.h"
#include "MenuV2.h"
#include "variable.h"
#include "func.h"
#include "FlashWrapper.h"

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

//camera
#define Width 80
#define Height 60
bool camptr[Height][Width];

int mode = 0;

carState magState = kNormal;

const uint8_t cycle = 12;
float loopSpeed = 9, highSpeed = 8.8, alignSpeed = 9;
float speed = highSpeed;
float yTarget = 0;
bool approaching = false, isFirst = false, firstArrived = false, secondArrived = false, USsent = false;
bool left_loop = false;
uint8_t leaveCount = 0;
uint32_t lastTime = 0, approachTime = 0;
float l1 = 100,l2 = 100,r1 = 100,r2 = 100, ry;

inline bool ret_cam_bit(int x, int y, const Byte* camBuffer) {

	Byte t = ((camBuffer[y * 10 + x / 8] >> (7 - (x % 8))) & 1);
//	t = (t << 3) || (t >> 5); // temp fix
	return t; //return 1 if black
}

int loop_control(int state, bool &is_loop, Mag* magnetic, int &camera_control, float &camera_angle, int master_edge_size, int slave_edge_size, int m_edge_xmid, int s_edge_xmid, int master_corner_size, int slave_corner_size, float master_slope, float slave_slope){
	static bool cameraReady = false, magReady = false;
	static float lastY = 0, rateY = 0, currY = 0, prevRateY = 0;
	switch(state){
	case 0:
		if(is_loop){
			state = 1;
			magState = kEnter;
			yTarget = magnetic->GetYLinear();
			if(master_slope < (-slave_slope)){
				left_loop = true;
			}else{
				left_loop = false;
			}
			magReady = false;
			l1 = master_slope;
			r1 = slave_slope;
			lastY = magnetic->GetYSum();
		}
		break;
	case 1:
		currY = magnetic->GetYSum();
		prevRateY = rateY;
		rateY = currY - lastY;
		rateY = 0.5*rateY + 0.5*prevRateY;
		ry = rateY;
		lastY = currY;
		if (rateY > 0){
			magReady = true;
		}
		if (magReady && magnetic->GetYSum() > 27){
			state = 2;
			camera_control = true;
			l2 = master_slope;
			r2 = slave_slope;
		}
		break;
	case 2:
		if(left_loop){
			float difference = (40 - m_edge_xmid)/40.0;
			if(difference<0.5){
				camera_angle = difference*100;
			}
			else{
				camera_angle = difference*200;
			}
			if(slave_edge_size<4){
				state = 3;
				cameraReady = false;
				magReady = false;
			}
		}else{
			float difference = (40 - s_edge_xmid)/40.0;
			if(difference>-0.5){
				camera_angle = difference*100;
			}
			else{
				camera_angle = difference*200;
			}
			if(master_edge_size<4){
				state = 3;
				cameraReady = false;
				magReady = false;
			}
		}
		break;
	case 3:
		if(left_loop){
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
				cameraReady = true;
			}
		}
		if (magnetic->GetXSum() < 125){
			magReady = true;
		}
		if(cameraReady && magReady){
			state = 4;
			camera_control = false;
			magState = carState::kLoop;
		}
		break;
	case 4:
		if(magnetic->outLoop()){
			state = 5;
			magState = carState::kExitLoop;
			lastY = magnetic->GetYSum();
		}
		break;
	case 5:
		currY = magnetic->GetYSum();
		prevRateY = rateY;
		rateY = currY - lastY;
		rateY = 0.9*rateY + 0.1*prevRateY;
		ry = rateY;
		lastY = currY;
		if (rateY < 0.5){
			state = 6;
			yTarget = magnetic->GetYLinear();
			magState = carState::kOutLoop;
		}
		break;
	case 6:
		if (magnetic->GetXSum() < 140 && magnetic->GetYSum() < 35){
			state = 0;
			camera_control = false;
			is_loop = false;
			magState = kNormal;
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

	FlashWrapper flashWrapper;
//	use this one instead la flashWrapper.imainboardID
//	int boardID = flashWrapper.getBoardID();

	int cam_contrast = 0x40;
	int pre_contrast = 0x40;

	Ov7725 camera(myConfig::getCameraConfig(Width, Height, 0));
	camera.Start();
	camera.ChangeSecialDigitalEffect(0x00, cam_contrast);

	Led led0(myConfig::GetLedConfig(0));
	Led led1(myConfig::GetLedConfig(1));

	led0.SetEnable(1);
	led1.SetEnable(1);

	Mag mag;

	BatteryMeter batteryMeter(myConfig::GetBatteryMeterConfig(flashWrapper.getBoardID()));
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
	PID servoPIDAlign(0, 0);
	PID cameraPID(0,0);
	PID left_motorPID(0, 0, 0, &encoderL, false);
	PID right_motorPID(0, 0, 0, &encoderR, true);

	Edge left_edge(false);

	//pid value
	float left_motor_pid[3],right_motor_pid[3],x_servo_pd[2],y_servo_pd[2],align_servo_pd[2];
	bool forwardL, forwardR;
	uint16_t middleServo, leftServo, rightServo;
	float angle = 0, angleX = 0, angleY = 0;

	//flash

	if (flashWrapper.imainboardID == 1) {
		left_motor_pid[0] = 0.62;
		left_motor_pid[1] = 0.03;
		left_motor_pid[2] = 0.008;

		right_motor_pid[0] = 0.62;
		right_motor_pid[1] = 0.03;
		right_motor_pid[2] = 0.008;

//	    x_servo_pd[0] = 9700;
//	    x_servo_pd[1] = 820000;
//
//	    y_servo_pd[0] = 3.85;
//	    y_servo_pd[1] = 228.73725;

		x_servo_pd[0] = 12000;
		x_servo_pd[1] = 1800000;

		y_servo_pd[0] = 8;
		y_servo_pd[1] = 280;

		align_servo_pd[0] = 5.8;
		align_servo_pd[1] = 750;

		forwardL = true;
		forwardR = false;

		middleServo = 1045;
		leftServo = 1340;
		rightServo = 710;

		mag.InitMag(1);
//		mag.InitMag(1, &flashWrapper);
	} else {
	    left_motor_pid[0] = 0.62;
	    left_motor_pid[1] = 0.03;
	    left_motor_pid[2] = 0.008;

	    right_motor_pid[0] = 0.62;
	    right_motor_pid[1] = 0.03;
	    right_motor_pid[2] = 0.008;

	    x_servo_pd[0] = 9700;
	    x_servo_pd[1] = 2900000;

	    y_servo_pd[0] = 8.8;
	    y_servo_pd[1] = 50;

	    align_servo_pd[0] = 5.8;
	    align_servo_pd[1] = 750;

		forwardL = true;
		forwardR = false;

		middleServo = 830;
		leftServo = 1145;
		rightServo = 540;

		mag.InitMag(2);
//		mag.InitMag(2, &flashWrapper);
	}

	float camera_angle = 0;
	float lastServo = 0;

	uint8_t cycleTime = 0;
	float encoderLval, encoderRval;
	float voltL, voltR;
	int32_t powerL, powerR;
	float batterySum = 0;
	float batteryCount = 0;

	// below sync data to the computer side
	// DualCar_UART_Sim << a slim ver only capable of sending bool
	// just change it to DualCar_UART if u want other function, it will do no harm
	// due to other bug, i created it, but then i realized the act is useless...
	DualCar_UART uart0(2); // << BT related
	DualCar_UART_Sim uartToAnotherCar(1); // << BT related

	// joseph: chnaged the 3 below to false, better send implicitly
	uartToAnotherCar.add(DualCar_UART_Config::BOOLEAN::b0, &approaching, false);
	uartToAnotherCar.add(DualCar_UART_Config::BOOLEAN::b1, &firstArrived, false);
	uartToAnotherCar.add(DualCar_UART_Config::BOOLEAN::b2, &secondArrived, false);
	uartToAnotherCar.add(DualCar_UART_Config::BOOLEAN::b3, &isFirst, false);
	uartToAnotherCar.add(DualCar_UART_Config::BOOLEAN::b4, &USsent, false);


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
//	uart0.add(DualCar_UART::FLOAT::f25, &angle, false);

	uart0.parseValues();
	uartToAnotherCar.parseValues();

	DistanceModule UltrasonicSensor([&](float distanceInCm) {
		if (distanceInCm < 30) {
			if(approaching && secondArrived){
				approaching = false;
				firstArrived = false;
				secondArrived = false;
				isFirst = false;
				uartToAnotherCar.Send_bool(DualCar_UART_Config::BOOLEAN::b4, true);
				approachTime = lastTime;
			}
		}
	});

	//dotted lines
	int accumulate_corner = 0;
	uint8_t dot_time = 0;
	bool start_count_corner = false;

	bool isTunning = true;

	//for loop ver2
	bool in_loop = false;
	int camera_control = false;
	int current_loop_state = 0;
	//

	//menu v2
	DualCarMenu menuV2(&lcd, &writer, Width, Height);
	DualCarMenu::SubMenu* current_page = &menuV2.home_page;

	menuV2.AddItem((char *) "start", &flashWrapper.imainboardID, &(menuV2.home_page), true);
	menuV2.AddItem((char *) "OpenMotor", menuV2.home_page.submenu_items[0].next_page, true);
	menuV2.AddItem((char *) "CloseMotor", menuV2.home_page.submenu_items[0].next_page, true);

	int left_corner_size = 0;
	int right_corner_size = 0;
	int dotted_lineV2 = 0;
	menuV2.AddItem((char *) "camera", &(menuV2.home_page), true);
	menuV2.AddItem((char *) "Ctr", &cam_contrast, menuV2.home_page.submenu_items[1].next_page, true);
	menuV2.AddItem((char *) "Lcorner", &left_corner_size, menuV2.home_page.submenu_items[1].next_page, false);
	menuV2.AddItem((char *) "Rcorner", &right_corner_size, menuV2.home_page.submenu_items[1].next_page, false);
	menuV2.AddItem((char *) "ac", &accumulate_corner, menuV2.home_page.submenu_items[1].next_page, false);


	menuV2.AddItem((char *) "Magnetic", &(menuV2.home_page), true);
	int mag_xL = mag.GetMag(Mag::magPos::x_left);
	int mag_xR = mag.GetMag(Mag::magPos::x_right);
	int mag_yL = mag.GetMag(Mag::magPos::y_left);
	int mag_yR = mag.GetMag(Mag::magPos::y_right);
	int mag_xSum = mag.GetXSum();
	int mag_ySum = mag.GetYSum();
	menuV2.AddItem((char *) "XL", &mag_xL, menuV2.home_page.submenu_items[2].next_page, false);
	menuV2.AddItem((char *) "XR", &mag_xR, menuV2.home_page.submenu_items[2].next_page, false);
	menuV2.AddItem((char *) "YL", &mag_yL, menuV2.home_page.submenu_items[2].next_page, false);
	menuV2.AddItem((char *) "YR", &mag_yR, menuV2.home_page.submenu_items[2].next_page, false);
	menuV2.AddItem((char *) "SX", &mag_xSum, menuV2.home_page.submenu_items[2].next_page, false);
	menuV2.AddItem((char *) "SY", &mag_ySum, menuV2.home_page.submenu_items[2].next_page, false);
	menuV2.AddItem((char *) "AX", &angleX, menuV2.home_page.submenu_items[2].next_page, false);
	menuV2.AddItem((char *) "AY", &angleY, menuV2.home_page.submenu_items[2].next_page, false);
	menuV2.AddItem((char *) "A", &angle, menuV2.home_page.submenu_items[2].next_page, false);

	menuV2.AddItem((char *) "loop", &(menuV2.home_page), true);
	menuV2.AddItem((char *) "state", &current_loop_state, menuV2.home_page.submenu_items[3].next_page, false);
	menuV2.AddItem((char *) "cam_con", &camera_control, menuV2.home_page.submenu_items[3].next_page, false);
	menuV2.AddItem((char *) "L1", &l1, menuV2.home_page.submenu_items[3].next_page, false);
	menuV2.AddItem((char *) "L2", &l2, menuV2.home_page.submenu_items[3].next_page, false);
	menuV2.AddItem((char *) "R1", &r1, menuV2.home_page.submenu_items[3].next_page, false);
	menuV2.AddItem((char *) "R2", &r2, menuV2.home_page.submenu_items[3].next_page, false);

	int intmagState = (int)magState;
	int* pmagState = &intmagState;
	menuV2.AddItem((char *) "MagSt", pmagState, &(menuV2.home_page), false);

	menuV2.AddItem((char *) "other", &(menuV2.home_page), true);
	float distance = UltrasonicSensor.getDistance();
	menuV2.AddItem((char *) "Dist", &(distance), menuV2.home_page.submenu_items[5].next_page, false);
	menuV2.AddItem((char *) "Volt", &(batteryVoltage), menuV2.home_page.submenu_items[5].next_page, false);
	menuV2.AddItem((char *) "EncL", &(encoderLval), menuV2.home_page.submenu_items[5].next_page, false);
	menuV2.AddItem((char *) "EncR", &(encoderRval), menuV2.home_page.submenu_items[5].next_page, false);
	int mpu_data = 0;
	int* pmpu_data = &mpu_data;
	menuV2.AddItem((char *) "mpu", pmpu_data, menuV2.home_page.submenu_items[5].next_page, false);

	menuV2.AddItem((char *) "Calibrate", &(menuV2.home_page), true);
	menuV2.AddItem((char *) "Reset", [&mag](){mag.ResetMag();}, menuV2.home_page.submenu_items[6].next_page, false);
	int mXL, mXR, mYL, mYR;
	int rXL, rXR, rYL, rYR;
	menuV2.AddItem((char *) "rXL", &(rXL), menuV2.home_page.submenu_items[6].next_page, false);
	menuV2.AddItem((char *) "rXR", &(rXR), menuV2.home_page.submenu_items[6].next_page, false);
	menuV2.AddItem((char *) "rYL", &(rYL), menuV2.home_page.submenu_items[6].next_page, false);
	menuV2.AddItem((char *) "rYR", &(rYR), menuV2.home_page.submenu_items[6].next_page, false);
	menuV2.AddItem((char *) "mXL", &(mXL), menuV2.home_page.submenu_items[6].next_page, false);
	menuV2.AddItem((char *) "mXR", &(mXR), menuV2.home_page.submenu_items[6].next_page, false);
	menuV2.AddItem((char *) "mYL", &(mYL), menuV2.home_page.submenu_items[6].next_page, false);
	menuV2.AddItem((char *) "mYR", &(mYR), menuV2.home_page.submenu_items[6].next_page, false);

	menuV2.AddItem((char *) "Flash", &(menuV2.home_page), true);
	menuV2.AddItem((char *) "mainboard", &flashWrapper.imainboardID, menuV2.home_page.submenu_items[7].next_page, true);
	menuV2.AddItem((char *) "SaveConfig", [&flashWrapper](){
		flashWrapper.saveConfigFromMenu();
	}, menuV2.home_page.submenu_items[7].next_page, false);
	menuV2.AddItem((char *) "SaveFlash",[&flashWrapper, &camera](){
		flashWrapper.writeFlash();
	}, menuV2.home_page.submenu_items[7].next_page, false);
	menuV2.AddItem((char *) "LoadFlash", [&flashWrapper](){
		flashWrapper.readFlash();
	}, menuV2.home_page.submenu_items[7].next_page, false);

	menuV2.AddItem((char *) "BT", &(menuV2.home_page), true);
	menuV2.AddItem((char *) "connectA", [&uartToAnotherCar](){
		uartToAnotherCar.HM10Func(DualCar_UART_Config::HM10ACT::connectToA);
	}, menuV2.home_page.submenu_items[8].next_page, false);
	menuV2.AddItem((char *) "connectB", [&uartToAnotherCar](){
		uartToAnotherCar.HM10Func(DualCar_UART_Config::HM10ACT::connectToB);
	}, menuV2.home_page.submenu_items[8].next_page, false);
	menuV2.AddItem((char *) "connectC", [&uartToAnotherCar](){
		uartToAnotherCar.HM10Func(DualCar_UART_Config::HM10ACT::connectToC);
	}, menuV2.home_page.submenu_items[8].next_page, false);
	menuV2.AddItem((char *) "connectD", [&uartToAnotherCar](){
		uartToAnotherCar.HM10Func(DualCar_UART_Config::HM10ACT::connectToD);
	}, menuV2.home_page.submenu_items[8].next_page, false);
	menuV2.AddItem((char *) "setAsSlave", [&uartToAnotherCar](){
		uartToAnotherCar.HM10Func(DualCar_UART_Config::HM10ACT::setAsSlave);
	}, menuV2.home_page.submenu_items[8].next_page, false);

	menuV2.AddItem((char *) "test", &(menuV2.home_page), true);

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
	servoPIDAlign.setkP(align_servo_pd[0]);
	servoPIDAlign.setkD(align_servo_pd[1]);
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
//				uart0.Send_float(DualCar_UART::FLOAT::f13, ry);
				uart0.Send_float(DualCar_UART::FLOAT::f13, mag.GetYLinear());
				uart0.Send_float(DualCar_UART::FLOAT::f21, servoPIDx.getpTerm());
				uart0.Send_float(DualCar_UART::FLOAT::f22, servoPIDx.getdTerm());
				uart0.Send_float(DualCar_UART::FLOAT::f23, servoPIDy.getpTerm());
				uart0.Send_float(DualCar_UART::FLOAT::f24, servoPIDy.getdTerm());
			}

			if (USsent) {
				if (firstArrived || secondArrived){
					uartToAnotherCar.Send_bool(DualCar_UART::BOOLEAN::b4, true);
				}
				approaching = false;
				isFirst = false;
				firstArrived = false;
				secondArrived = false;
				USsent = false;
				approachTime = lastTime;
			}

			uart0.RunEveryMS();
			uartToAnotherCar.RunEveryMS();
			led0.SetEnable(lastTime % 100 < 50);
			led1.SetEnable(lastTime % 100 < 50);
//			led0.SetEnable(!approaching);
//			led1.SetEnable(!isFirst);
			mag.TakeSample();
			batterySum += batteryMeter.GetVoltage();
			batteryCount++;

			if (lastTime - on9lastMain >= cycle) {
#ifdef temp_cam_fix
				const Byte* camBuffer_Original = camera.LockBuffer();
				camera.UnlockBuffer();

				Byte * camBuffer = new Byte [Width * Height / 8];
				for (uint16_t i = 0; i < Width * Height / 8; i++) {
					Byte t = *(camBuffer_Original + i);
					*(camBuffer + i) = t << 3 | t >> 5;
				}
#else
				const Byte* camBuffer_Original = camera.LockBuffer();
				camera.UnlockBuffer();
#endif

				if(pre_contrast!=cam_contrast){
					camera.ChangeSecialDigitalEffect(0x00, cam_contrast);
					pre_contrast = cam_contrast;
				}

				mag.Update();
				on9lastMain = lastTime;
				if (mag.noMagField() && current_page->identity == "OpenMotor") {
					speed = 0;
				}

				//changes state for alignment
				mag.CheckState(lastTime, approachTime, magState, speed, approaching, isFirst, firstArrived, secondArrived);

				buzz.SetBeep(approaching);

				if (approaching){
					if (isFirst && firstArrived){
						uartToAnotherCar.Send_bool(DualCar_UART::BOOLEAN::b1, true);
					} else if (!isFirst && secondArrived){
						uartToAnotherCar.Send_bool(DualCar_UART::BOOLEAN::b2, true);
					}
				}

				if (isTunning){
					servoPIDx.setkP(x_servo_pd[0]);
					servoPIDx.setkD(x_servo_pd[1]);
					servoPIDy.setkP(y_servo_pd[0]);
					servoPIDy.setkD(y_servo_pd[1]);
					servoPIDAlign.setkP(align_servo_pd[0]);
					servoPIDAlign.setkD(align_servo_pd[1]);
					left_motorPID.setkP(left_motor_pid[0]);
					left_motorPID.setkI(left_motor_pid[1]);
					left_motorPID.setkD(left_motor_pid[2]);
					right_motorPID.setkP(right_motor_pid[0]);
					right_motorPID.setkI(right_motor_pid[1]);
					right_motorPID.setkD(right_motor_pid[2]);
				}

				//LOOP v2
				master_edge = left_edge.check_edge(camBuffer, 25, 60);
				vector<Corner> master_corner;
				master_corner = check_cornerv2(camBuffer, 25, 60, master_edge);
				slave_corner = m_master_bluetooth.get_slave_corner();
				if((mpu_data > 3000 || mpu_data <- 3000)){
					bumpy_road = true;
				}
				else{
					bumpy_road = false;
				}

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

				if(slave_edge_size<2){
					s_edge_xmid = 80;
				}

				if (in_loop){
					current_loop_state = loop_control(current_loop_state, in_loop, &mag, camera_control, camera_angle, master_edge.size(), slave_edge_size, m_edge_xmid, s_edge_xmid, master_corner.size(), slave_corner.size(),master_slope, slave_slope);
				}

				if(camera_control){
					angle = camera_angle;
				} else{
					if (current_page->identity == "Calibrate") {
						angle = 0;
					} else{
						angle = mag.GetAngle(servoPIDx, servoPIDy, servoPIDAlign, angleX, angleY, magState, left_loop, in_loop, yTarget);
					}
				}

				//alignment
				//////use this as long as it sees at least one corner
				vector<pair<int,int>> junction;
				int junction_array[35];
				for(int i=0; i<35; i++){
					junction_array[i] = 0;
				}
				for(int i=0; i<master_edge.size(); i++){
					if(junction.size() == 0){
						junction.push_back(make_pair(master_edge[0].first,master_edge[0].second));
						continue;
					}
					bool found = false;
					for(int j=0; j<junction.size();j++){
						if((master_edge[i].second==junction[j].second)&&(abs(junction[j].first-master_edge[i].first)>5)){
							junction[j].first = master_edge[i].first;
							junction_array[master_edge[i].second-25]++;
							found = true;
							break;
						}
						else if((master_edge[i].second==junction[j].second)&&(abs(junction[j].first-master_edge[i].first)<=5)){
							found = true;
							break;
						}
					}
					if((master_edge[i].second<60)&&(master_edge[i].second>=25)&&(!found)){
						junction_array[master_edge[i].second-25] += 1;
						junction.push_back(make_pair(master_edge[i].first,master_edge[i].second));
					}
				}

				for(int i=0; i<35; i++){
					if(junction_array[i]>2){
						dotted_lineV2 = true;
						break;
					}
				}

				//

				if((dotted_lineV2)&&(master_corner.size()>0)
						&& (slave_corner.size()>0) && (!start_count_corner)&&(!bumpy_road)&&(!in_loop)&&(!approaching)){
					dot_time = 0;
					buzz.SetNote(440);
					buzz.SetBeep(true);
					start_count_corner = true;
				}


				if(start_count_corner){
					dot_time++;
					if(dot_time == 10 && accumulate_corner > 20){
						buzz.SetBeep(false);
						start_count_corner = false;
						if(!approaching && !mag.isTwoLine() && mag.unlikelyCrossRoad() && (lastTime - approachTime >= 10000 || approachTime == 0)){
							approaching = true;
							if (!firstArrived){
								isFirst = true;
								firstArrived = true;
							}
						}
						accumulate_corner = 0;
						dot_time = 0;
					}else if (dot_time == 10){
						accumulate_corner = 0;
						dot_time = 0;
					}else{
						accumulate_corner += master_corner.size();
						accumulate_corner += slave_corner.size();
					}
				}
				//

				(*pmpu_data) = m_master_bluetooth.get_mpu_data();



				angle = 0.8*angle + 0.2*lastServo;
				angle = Max(rightServo-middleServo, Min(leftServo-middleServo, angle));
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
				*pmagState = (int)magState;
				if (current_page->identity == "Magnetic"){
					mag_xL = mag.GetMag(Mag::magPos::x_left);
					mag_xR = mag.GetMag(Mag::magPos::x_right);
					mag_yL = mag.GetMag(Mag::magPos::y_left);
					mag_yR = mag.GetMag(Mag::magPos::y_right);
					mag_xSum = mag.GetXSum();
					mag_ySum = mag.GetYSum();
				}
				if (current_page->identity == "Calibrate"){
					mag.Calibrate();
					rXL = mag.GetRaw(Mag::magPos::x_left);
					rXR = mag.GetRaw(Mag::magPos::x_right);
					rYL = mag.GetRaw(Mag::magPos::y_left);
					rYR = mag.GetRaw(Mag::magPos::y_right);
					if (rXL > 20){
						mXL = mag.GetMax(Mag::magPos::x_left);
					} else{
						mXL = mag.GetMin(Mag::magPos::x_left);
					}
					if (rXR > 20){
						mXR = mag.GetMax(Mag::magPos::x_right);
					} else{
						mXR = mag.GetMin(Mag::magPos::x_right);
					}
					if (rYL > 20){
						mYL = mag.GetMax(Mag::magPos::y_left);
					} else{
						mYL = mag.GetMin(Mag::magPos::y_left);
					}
					if (rYR > 20){
						mYR = mag.GetMax(Mag::magPos::y_right);
					} else{
						mYR = mag.GetMin(Mag::magPos::y_right);
					}
//					buzz.SetBeep(false);
				}
				distance = UltrasonicSensor.getDistance();
				left_corner_size = master_corner.size();
				right_corner_size = slave_corner.size();
				menuV2.SetCamBuffer(camBuffer);
				menuV2.SetEdge(master_edge);
				menuV2.SetCorner(master_corner);
				current_page = menuV2.PrintSubMenu(current_page);
				batteryVoltage = batterySum/batteryCount;
				if(current_page->identity == "OpenMotor"){
					voltR = right_motorPID.getPID();
					voltL = left_motorPID.getPID();
					powerR = voltR/batteryVoltage*1000;
					powerL = voltL/batteryVoltage*1000;
					if (powerR > 0) {
							right_motor.SetClockwise(forwardR);
							right_motor.SetPower(Min(powerR,1000));
					} else {
						right_motor.SetClockwise(!forwardR);
						right_motor.SetPower(Min(-powerR,1000));
					}
					if (powerL > 0) {
						left_motor.SetClockwise(forwardL);
						left_motor.SetPower(Min(powerL,1000));
					} else {
						left_motor.SetClockwise(!forwardL);
						left_motor.SetPower(Min(-powerL,1000));
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

#ifdef temp_cam_fix
				delete[] camBuffer;
#endif

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
