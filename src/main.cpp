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
#include "DualCar_UART.h"
//#include "bt.h"

#include "libsc/passive_buzzer.h"

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

int max(int a, int b){return a>b?a:b;}
int min(int a, int b){return a<b?a:b;}

bool IsOneLine(uint8_t maxVal, uint8_t leftVal, uint8_t rightVal, float multi = 1.5){
	if (max(leftVal,rightVal) < maxVal && (leftVal+rightVal) < maxVal*multi){
		return true;
	}
	return false;
}

bool IsTwoLine(uint8_t maxVal, uint8_t leftVal, uint8_t rightVal, float multi = 1.6){
	if (max(leftVal,rightVal) > maxVal && (leftVal+rightVal) > maxVal*multi && min(leftVal,rightVal) > maxVal*0.4){
		return true;
	}
	return false;
}

int main() {


    System::Init();

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

	PassiveBuzzer::Config config;
	PassiveBuzzer buzz(config);
	bool sound = false;
	buzz.SetBeep(sound);
	buzz.SetNote(523);

	Servo servo(myConfig::GetServoConfig());
	AlternateMotor motorL(myConfig::GetMotorConfig(0));
	motorL.SetClockwise(false);
	AlternateMotor motorR(myConfig::GetMotorConfig(1));
    St7735r lcd(myConfig::GetLcdConfig());
    LcdTypewriter writer(myConfig::GetWriterConfig(&lcd));
    LcdConsole console(myConfig::GetConsoleConfig(&lcd));
    lcd.SetRegion(Lcd::Rect(0,0,128,160));

    DirEncoder LEncoder(myConfig::GetEncoderConfig(1));
    DirEncoder REncoder(myConfig::GetEncoderConfig(0));
    PID servoPIDCurv(6275,69);
    PID servoPIDStraight(1800,70);
    PID servoPIDCurve(6275,69);//for straight123
    PID servoPIDOneStraight(4,100);//10
    PID servoPIDOneCurve(-18,240);//-24,240
    PID motorLPID(0.145,0.0,1.4, &LEncoder);
    PID motorRPID(0.145,0.0,1.4, &REncoder);
    const uint8_t cycle = 8;
    const float lowSpeed = 4*cycle, highSpeed = 6*cycle;
    float speed = highSpeed;
	float lastServo, frontLinear, midLinear;
//    bt mBT(&servoPIDCurv, &servoPIDStraight, &motorLPID, &motorRPID, &speed, &frontLinear);

	typedef enum {
		normal = 0,
		nearLoop,
		straight1,
		straight2,
		turning,
		inLoop,
		outLoop
	}carState;
	carState state = normal;

	bool start = false;
	bool leftLoop = true, bigVal = false;
	uint16_t filterSum0 = 0, filterSum1 = 0, filterSum2 = 0, filterSum3 = 0, filterSum4 = 0, filterSum5 = 0;
	uint8_t filterCounter = 0;
	uint32_t lastTime = 0, stateTime = 0;
	uint8_t front_left, front_right, mid_left, mid_right, back_left, back_right;

	const uint16_t middleServo = 865, leftServo = 1180, rightServo = 550;
	float angle = middleServo;

	uint8_t oneLineMax = 80, equalMin = 55, equalMax = 51;
	uint8_t maxLeft = 0,minLeft = 100,maxRight = 0,minRight = 100;
	uint8_t count1, count4;

    Joystick js(myConfig::GetJoystickConfig(Joystick::Listener([&](const uint8_t id, const Joystick::State State){
    	if (State == Joystick::State::kSelect){
    		if (start){
    			start = false;
    			motorLPID.setDesiredVelocity(0);
    			motorRPID.setDesiredVelocity(0);
				motorL.SetPower(0);
				motorR.SetPower(0);
    		}
    		else{
				start = true;
				motorLPID.setDesiredVelocity(speed);
				motorRPID.setDesiredVelocity(speed);
    		}
		}
    	else if (State == Joystick::State::kUp){
    		state = (carState)(((int)state+1) % 7);
    	}
    	else{
    		sound = !sound;
    		buzz.SetBeep(sound);
    	}
    })));

	DualCar_UART uart0; // << BT related
	uart0.add(DualCar_UART::UINT8_T::u0, &front_left, true); // << BT false:changed by computer
	uart0.add(DualCar_UART::UINT8_T::u1, &front_right, true); // << BT true:changing variable
	uart0.add(DualCar_UART::UINT8_T::u2, &mid_left, true);
	uart0.add(DualCar_UART::UINT8_T::u3, &mid_right, true);
	uart0.add(DualCar_UART::FLOAT::f0, &frontLinear, true);
	uart0.add(DualCar_UART::FLOAT::f1, &midLinear, true);
	uart0.parseValues(); // << BT related

	servo.SetDegree(middleServo);
    while(1){
    	if(System::Time() != lastTime){
    		lastTime = System::Time();

    		uart0.RunEveryMS(); // << BT related

//			mid_left = mag0.GetResult();
//			mid_right = mag1.GetResult();
//			front_left = mag2.GetResult();
//			front_right = mag3.GetResult();

    		if (lastTime % cycle == 0){
				mid_left = filterSum0/filterCounter;
				mid_right = filterSum1/filterCounter;
				front_left = filterSum2/filterCounter;
				front_right = filterSum3/filterCounter;
				back_left = filterSum4/filterCounter;
				back_right = filterSum5/filterCounter;
				filterSum0 = 0;
				filterSum1 = 0;
				filterSum2 = 0;
				filterSum3 = 0;
				filterSum4 = 0;
				filterSum5 = 0;
				filterCounter = 0;

				if (start && max(max(max(front_left,front_right),mid_left),mid_right) < 20){
					start = false;
					state = normal;
					motorL.SetPower(0);
					motorR.SetPower(0);
	    			motorLPID.setDesiredVelocity(0);
	    			motorRPID.setDesiredVelocity(0);
				}

//				minLeft = min(minLeft, front_left);
//				maxLeft = max(maxLeft, front_left);
//				minRight = min(minRight, front_right);
//				maxRight = max(maxRight, front_right);

				if (!IsTwoLine(oneLineMax, front_left, front_right) ){
					if (state == turning && (leftLoop && front_left>front_right*1.45 || !leftLoop && front_right>front_left*1.45)){
						state = inLoop;
						speed = (lowSpeed+highSpeed)/2;
						stateTime = lastTime;
						bigVal = false;
					}
					else if (IsOneLine(oneLineMax, front_left, front_right) && IsOneLine(oneLineMax, mid_left, mid_right) && (state == inLoop && bigVal || state == outLoop)){
						state = normal;
						stateTime = lastTime;
						speed = highSpeed;
					}
				}
				else{
					if (!IsTwoLine(oneLineMax, mid_left, mid_right)){
						if (state == normal){
							state = nearLoop;
							speed = lowSpeed;
							stateTime = lastTime;
							if (front_left-mid_left > front_right-mid_right){
								leftLoop = true;
							}
							else{
								leftLoop = false;
							}
						}
					}
					else{
						if (state == nearLoop){
							state = straight1;
							stateTime = lastTime;
						}
						else if (state == inLoop){
							state = outLoop;
							stateTime = lastTime;
						}
						else if (state == straight1 && abs(front_left-front_right) < 5){
							state = straight2;
							stateTime = lastTime;
						}
						else if (state == straight2 && (leftLoop && front_left*0.9>mid_left || !leftLoop && front_right*0.9>mid_right)){//0.9
							state = turning;
							stateTime = lastTime;
						}
					}
				}


				if (state == normal){
					if (front_left < 15 || front_right < 15){
						angle = lastServo*1.7;
					}
					else{
						frontLinear = 1/(float)front_left-1/(float)front_right;
						midLinear = 1/(float)mid_left-1/(float)mid_right;
						if(frontLinear-midLinear >= 0.01 || frontLinear-midLinear <= -0.01 || frontLinear >= 0.035 || frontLinear <= - 0.035){//0.003
							angle = servoPIDCurv.getPID(0.0,frontLinear);
						}else{
							angle = servoPIDStraight.getPID(0.0,frontLinear);
						}
						lastServo = angle;
						if (front_left == front_right){
							equalMin = min(equalMin,front_left);
							equalMax = max(equalMax,front_left);
						}
					}
					led3.SetEnable(0);
					led2.SetEnable(1);
				}
				else if (state == nearLoop){
					if (leftLoop){
						angle = servoPIDOneStraight.getPID((equalMax+equalMin)/2, mid_right);
					}
					else{
						angle = -servoPIDOneStraight.getPID((equalMax+equalMin)/2, mid_left);
					}
					led3.SetEnable(0);
					led2.SetEnable(0);
				}
				else if (state == straight1 || state == straight2){
					midLinear = 2*(1/(float)mid_left-1/(float)mid_right);
					frontLinear = 2*(1/(float)front_left-1/(float)front_right);
					if (state == straight1){
						angle = servoPIDCurve.getPID(0.0,frontLinear);
					}else{
						angle = servoPIDCurve.getPID(0.0,midLinear);
					}
					if (straight1 == state){
						led3.SetEnable(1);
						led2.SetEnable(0);
						led1.SetEnable(1);
					}
					else if (state == straight2){
						led1.SetEnable(0);
					}
				}
				else if (state == turning){
					if (leftLoop){
						if (mid_left < (equalMax+equalMin)/2){
							angle = 100;
						}
						else{
							angle = servoPIDOneCurve.getPID((equalMax+equalMin)/2, mid_left);
							lastServo = angle;
						}
					}
					else{
						if (mid_right < (equalMax+equalMin)/2){
							angle = -100;
						}
						else{
							angle = -servoPIDOneCurve.getPID((equalMax+equalMin)/2, mid_right);
							lastServo = angle;
						}
					}
					lastServo = angle;
					led3.SetEnable(1);
					led2.SetEnable(1);
				}
				else if (state == inLoop){
					if (!bigVal){
						if (front_left < 20 || front_right < 20){
							if (leftLoop){
								angle = 300;
							}
							else{
								angle = -300;
							}
						}
						else{
							frontLinear = 1/(float)front_left-1/(float)front_right;
							angle = servoPIDCurv.getPID(0.0,frontLinear);
							lastServo = angle;
						}
						if (max(front_left, front_right) > oneLineMax){
							bigVal = true;
						}
					}
					else{
						if (front_left < 50 || front_right < 50){
							if (leftLoop){
								angle = 300;
							}
							else{
								angle = -300;
							}
						}
						else{
							frontLinear = 2*(1/(float)front_left-1/(float)front_right);
							angle = servoPIDCurv.getPID(0.0,frontLinear);
						}
						led1.SetEnable(lastTime % 100 < 50);
					}
					led3.SetEnable(lastTime % 100 < 50);
					led2.SetEnable(lastTime % 100 < 50);
				}
				else if (state == outLoop){
					if (front_left < 20 || front_right < 20){
						if (leftLoop){
							angle = 300;
						}
						else{
							angle = -300;
						}
					}
					else{
						frontLinear = 1/(float)front_left-1/(float)front_right;
						angle = servoPIDCurv.getPID(0.0,frontLinear);
						lastServo = angle;
					}
					led3.SetEnable(lastTime % 100 < 50);
					led2.SetEnable(1);
				}
				led0.SetEnable(!leftLoop);

				angle += middleServo;
				angle = max(rightServo,min(leftServo,angle));
				servo.SetDegree(angle);

				if (angle>middleServo){
					motorLPID.setDesiredVelocity(speed*((angle-middleServo)/-7.5+90)/90);
					motorRPID.setDesiredVelocity(speed*((angle-middleServo)/8.5+90)/90);
				}
				else{
					motorRPID.setDesiredVelocity(speed*((middleServo-angle)/-7.5+90)/90);
					motorLPID.setDesiredVelocity(speed*((middleServo-angle)/8.5+90)/90);
				}

				if (start){
					motorR.SetPower(motorRPID.getPID());
					motorL.SetPower(motorLPID.getPID());
				}
//				mBT.sendVelocity();
				count4++;
			}

			filterCounter++;
			filterSum0 += mag0.GetResult();
			filterSum1 += mag1.GetResult();
			filterSum2 += mag2.GetResult();
			filterSum3 += mag3.GetResult();
//			filterSum0 += mid_left;
//			filterSum1 += mid_right;
//			filterSum2 += front_left;
//			filterSum3 += front_right;
			count1++;



			if (lastTime % 500 == 0){
				if (state != normal && lastTime-stateTime > 30000){//TODO change it back to 3000 after tuning
					state = normal;
					speed = highSpeed;
				}
			}

			if (lastTime % 100 == 0){
				if(!start){
					char c[10];
					lcd.SetRegion(Lcd::Rect(0,0,128,15));
					sprintf(c,"FL: %d", front_left);
					writer.WriteBuffer(c,10);
					lcd.SetRegion(Lcd::Rect(0,15,128,15));
					sprintf(c,"FR: %d", front_right);
					writer.WriteBuffer(c,10);
					lcd.SetRegion(Lcd::Rect(0,30,128,15));
					sprintf(c,"ML: %d",mid_left);
					writer.WriteBuffer(c,10);
					lcd.SetRegion(Lcd::Rect(0,45,128,15));
					sprintf(c,"MR: %d",mid_right);
					writer.WriteBuffer(c,10);
//					lcd.SetRegion(Lcd::Rect(0,60,128,15));
//					sprintf(c,"P: %d %d",count1,count4);
//					writer.WriteBuffer(c,10);
//					lcd.SetRegion(Lcd::Rect(0,75,128,15));
//					sprintf(c,"D: %d",count4);
//					writer.WriteBuffer(c,10);
					lcd.SetRegion(Lcd::Rect(0,90,128,15));
					sprintf(c,"A: %d",servo.GetDegree());
					writer.WriteBuffer(c,10);
//					lcd.SetRegion(Lcd::Rect(0,105,128,15));
//					sprintf(c,"S: %f",cc);
//					writer.WriteBuffer(c,10);
					lcd.SetRegion(Lcd::Rect(0,120,128,15));
					sprintf(c,"M: %f",frontLinear*100);
					writer.WriteBuffer(c,10);
////					if (state == normal){
////						lcd.FillColor(0xFF00);
////					}else if (state == nearLoop){
////						if (isLeft){
////							lcd.SetRegion(Lcd::Rect(0,120,64,15));
////						}
////						else{
////							lcd.SetRegion(Lcd::Rect(64,120,64,15));
////						}
////						lcd.FillColor(0x0FF0);
////					}else if (state == turning){
////						lcd.FillColor(0x00FF);
////					}else if (state == inLoop){
////						lcd.FillColor(0x0000);
////					}
					lcd.SetRegion(Lcd::Rect(0,135,128,15));
					sprintf(c,"M: %f",midLinear*100);
					writer.WriteBuffer(c, 10);
					count1 = 0;
					count4 = 0;
				}
			}
    	}
    }
    return 0;
}

