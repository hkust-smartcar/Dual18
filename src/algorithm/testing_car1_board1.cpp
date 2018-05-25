#include "algorithm/testing_car1_board1.h"

#include <cmath>
#include <cstring>
#include <cstdio>
#include <sstream>
#include <vector>
#include <cassert>

#include "libsc/dir_motor.h"
#include "libsc/dir_encoder.h"
#include "libsc/futaba_s3010.h"
#include "libsc/joystick.h"
#include "libsc/lcd_console.h"
#include "libsc/led.h"
#include "libsc/st7735r.h"
#include "libsc/system.h"
#include "libsc/k60/jy_mcu_bt_106.h"
#include "libsc/k60/ov7725.h"
//#include "libutil/misc.h"
//#include "libutil/incremental_pid_controller.h"
#include "config.h"
#include "PID.h"
#include "DualCar_UART.h"
#include "libsc/passive_buzzer.h"


using libsc::DirMotor;
using libsc::DirEncoder;
using libsc::FutabaS3010;
using libsc::Joystick;
using libsc::Lcd;
using libsc::LcdTypewriter;
using libsc::Led;
using libsc::St7735r;
using libsc::System;
using libsc::Timer;
using libsc::k60::JyMcuBt106;
using libsc::k60::Ov7725;
using libsc::k60::Ov7725Configurator;
using libsc::PassiveBuzzer;

namespace algorithm {
namespace testing {
namespace car1 {
namespace board1{


float inv_sqrt(float x)
{
	float xhalf = 0.5f * x;
	int i = *(int*) &x;
	i = 0x5f3759df - (i >> 1);
	x = *(float*) &i;
	x = x * (1.5f - xhalf * x * x);
	x = x * (1.5f - xhalf * x * x);
	return x;
}

float _sqrt(float x)
{
	return x * inv_sqrt(x);
}

int max(int a, int b)
{
	return a > b ? a : b;
}

int min(int a, int b)
{
	return a < b ? a : b;
}

bool IsOneLine(uint8_t maxVal, uint8_t leftVal, uint8_t rightVal, float multi = 1.5)
{
	if (max(leftVal,rightVal) < maxVal && (leftVal+rightVal) < maxVal*multi){
		return true;
	}
	return false;
}

bool IsTwoLine(uint8_t maxVal, uint8_t leftVal, uint8_t rightVal, float multi = 1.6)
{
	if (max(leftVal,rightVal) > maxVal && (leftVal+rightVal) > maxVal*multi && min(leftVal,rightVal) > maxVal*0.4){
		return true;
	}
	return false;
}

const uint8_t cycle = 8;
float lowSpeed = 5*cycle, highSpeed = 6.5*cycle;
float speed = highSpeed;
float lastServo, frontLinear, midLinear;
float raw_frontLinear, raw_midLinear;
float multiplier = 0.0;
float front_left = 1, front_right = 1, mid_left = 1, mid_right = 1, back_left = 1, back_right = 1;
uint8_t raw_front_left, raw_front_right, raw_mid_left, raw_mid_right;
uint8_t encoderLval, encoderRval, powerL, powerR;
float pCurve, dCurve, pStraight, dStraight, pMotor, iMotor, dMotor;
typedef enum {
	normal = 0,
	nearLoop,
	straight1,
	straight2,
	turning,
	inLoop,
	outLoop
}carState;

int main_car1_board1(bool debug_) {
	//initialize
	Led led0(Config::GetLedConfig(0));
	Led led1(Config::GetLedConfig(1));
	Led led2(Config::GetLedConfig(2));
	Led led3(Config::GetLedConfig(3));

	Adc mag0(Config::GetAdcConfig(Adc::Name::kAdc0Ad13));
	Adc mag1(Config::GetAdcConfig( Adc::Name::kAdc0Ad12));
	Adc mag2(Config::GetAdcConfig( Adc::Name::kAdc1Ad9));
	Adc mag3(Config::GetAdcConfig( Adc::Name::kAdc1Ad8));
	Adc mag4(Config::GetAdcConfig( Adc::Name::kAdc0Ad17));
	Adc mag5(Config::GetAdcConfig( Adc::Name::kAdc0Ad18));

	PassiveBuzzer::Config config;
	PassiveBuzzer buzz(config);

	Servo servo(Config::GetServoConfig());
	AlternateMotor motorL(Config::GetMotorConfig(0));
	AlternateMotor motorR(Config::GetMotorConfig(1));
	St7735r lcd(Config::GetLcdConfig());
	LcdTypewriter writer(Config::GetWriterConfig(&lcd));
	LcdConsole console(Config::GetConsoleConfig(&lcd));

	DirEncoder LEncoder(Config::GetEncoderConfig(1));
	DirEncoder REncoder(Config::GetEncoderConfig(0));

	carState state = normal;

	bool start = false;
	bool leftLoop = true, bigVal = false;
	uint16_t filterSum0 = 0, filterSum1 = 0, filterSum2 = 0, filterSum3 = 0, filterSum4 = 0, filterSum5 = 0;
	uint8_t filterCounter = 0;
	uint32_t lastTime = 0, stateTime = 0;

	const uint16_t middleServo = 865, leftServo = 1180, rightServo = 550;
	float angle = middleServo;

	uint8_t oneLineMax = 80, oneLineMin = 80, equalMin = 250, equalMax = 0;

	PID servoPIDStraight(2200,0.05);
	PID servoPIDCurve(6275,0);//for straight123
	PID servoPIDOneStraight(4,100);//10
	PID servoPIDOneCurve(-18,240);//-24,240
	PID motorLPID(0.145,0.0,1.35, &LEncoder);
	PID motorRPID(0.145,0.0,1.35, &REncoder);

    led0.SetEnable(1);
    led1.SetEnable(1);
    led2.SetEnable(1);
    led3.SetEnable(1);

    mag0.StartConvert();
    mag1.StartConvert();
    mag2.StartConvert();
    mag3.StartConvert();

    buzz.SetNote(523);
    motorL.SetClockwise(false);
    motorR.SetClockwise(true);
    lcd.SetRegion(Lcd::Rect(0,0,128,160));


    Joystick js(Config::GetJoystickConfig(Joystick::Listener([&](const uint8_t id, const Joystick::State State){
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
    })));

	DualCar_UART uart0; // << BT related

	uart0.add(DualCar_UART::FLOAT::f0, &frontLinear, true);
	uart0.add(DualCar_UART::FLOAT::f1, &midLinear, true);

	uart0.add(DualCar_UART::FLOAT::f3, &speed, false);
	uart0.add(DualCar_UART::FLOAT::f20, &pCurve, false);
	uart0.add(DualCar_UART::FLOAT::f21, &dCurve, false);
	uart0.add(DualCar_UART::FLOAT::f22, &pStraight, false);
	uart0.add(DualCar_UART::FLOAT::f23, &dStraight, false);

	uart0.add(DualCar_UART::FLOAT::f4, &multiplier, true);

	uart0.add(DualCar_UART::UINT8_T::u0, &encoderLval, true);
	uart0.add(DualCar_UART::UINT8_T::u1, &encoderRval, true);

	uart0.add(DualCar_UART::FLOAT::f10, &front_left, true); // << BT false:changed by computer
	uart0.add(DualCar_UART::FLOAT::f11, &front_right, true); // << BT true:changing variable
	uart0.add(DualCar_UART::FLOAT::f12, &mid_left, true);
	uart0.add(DualCar_UART::FLOAT::f13, &mid_right, true);
	uart0.add(DualCar_UART::UINT8_T::u14, &raw_front_left, true); // << BT false:changed by computer
	uart0.add(DualCar_UART::UINT8_T::u15, &raw_front_right, true); // << BT true:changing variable
	uart0.parseValues(); // << BT related

	servo.SetDegree(middleServo);


	//control cycle
    while(1){
    	if(System::Time() != lastTime){
    		lastTime = System::Time();

    		uart0.RunEveryMS(); // << BT related

    		if ((lastTime % cycle == 0)&&(filterCounter>0)){
    			raw_mid_left = round(1.0*filterSum0/filterCounter);
    			raw_mid_right = round(1.0*filterSum1/filterCounter);
    			raw_front_left = round(1.0*filterSum2/filterCounter);
    			raw_front_right = round(1.0*filterSum3/filterCounter);
    			oneLineMin  = min(raw_front_left, min(raw_front_right, min(raw_mid_left, min(raw_mid_right, oneLineMin))));
    			if (multiplier != 0){
    				mid_left = (raw_mid_left-oneLineMin)*multiplier;
					mid_right = (raw_mid_right-oneLineMin)*multiplier;
					front_left = (raw_front_left-oneLineMin)*multiplier;
					front_right = (raw_front_right-oneLineMin)*multiplier;
    			}
				filterSum0 = 0;
				filterSum1 = 0;
				filterSum2 = 0;
				filterSum3 = 0;
				filterSum4 = 0;
				filterSum5 = 0;
				filterCounter = 0;
				if (raw_front_left == raw_front_right){
					if (equalMin > 100 || (equalMin>raw_front_left && equalMin-raw_front_left < 5)){
						equalMin = raw_front_left;
    					multiplier = 50.0/(equalMin+equalMax-2*oneLineMin)*2;

					}
					if (equalMax < 10 || (equalMax<raw_front_left && raw_front_left-equalMax < 5)){
						equalMax = raw_front_left;
    					multiplier = 50.0/(equalMin+equalMax-2*oneLineMin)*2;
					}
				}

				if (start && max(max(max(front_left,front_right),mid_left),mid_right) < 20){
					start = false;
					state = normal;
					motorL.SetPower(0);
					motorR.SetPower(0);
	    			motorLPID.setDesiredVelocity(0);
	    			motorRPID.setDesiredVelocity(0);
				}

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

				raw_frontLinear = 1/(float)raw_front_left-1/(float)raw_front_right;
				raw_midLinear = 1/(float)raw_mid_left-1/(float)raw_mid_right;
				frontLinear = 1/(float)front_left-1/(float)front_right;
				midLinear = 1/(float)mid_left-1/(float)mid_right;

				if (state != normal){
					buzz.SetNote(698);
					state = normal;
				}
				if (state == normal){
					if (raw_front_left < 15 || raw_front_right < 15){
						angle = lastServo*1.7;
						buzz.SetNote(1318);
					}
					else{
						if(frontLinear-midLinear >= 0.01 || frontLinear-midLinear <= -0.01 || frontLinear >= 0.035 || frontLinear <= - 0.035){//0.003
							angle = servoPIDCurve.getPID(0.0,frontLinear);
							buzz.SetNote(587);
						}else{
							angle = servoPIDStraight.getPID(0.0,frontLinear);
							buzz.SetNote(523);
						}
						lastServo = angle;
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
					midLinear = 2*midLinear;
					frontLinear = 2*frontLinear;
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
							angle = servoPIDCurve.getPID(0.0,frontLinear);
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
							frontLinear = 2*frontLinear;
							angle = servoPIDCurve.getPID(0.0,frontLinear);
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
						angle = servoPIDCurve.getPID(0.0,frontLinear);
						lastServo = angle;
					}
					led3.SetEnable(lastTime % 100 < 50);
					led2.SetEnable(1);
				}


				angle += middleServo;
				angle = max(rightServo,min(leftServo,angle));
				servo.SetDegree(angle);

				motorRPID.setDesiredVelocity(speed);
				motorLPID.setDesiredVelocity(speed);

				if (start){
					powerR = motorRPID.getPID();
					powerL = motorLPID.getPID();
					motorR.SetPower(powerR);
					motorL.SetPower(powerL);
				}else{
					motorR.SetPower(0);
					motorL.SetPower(0);
				}

				encoderLval = LEncoder.GetCount();
				encoderRval = REncoder.GetCount();
			}

			filterCounter++;
			filterSum0 += mag0.GetResult();
			filterSum1 += mag1.GetResult();
			filterSum2 += mag2.GetResult();
			filterSum3 += mag3.GetResult();



			if (lastTime % 500 == 0){
				if (state != normal && lastTime-stateTime > 2000){
					state = normal;
					speed = highSpeed;
				}
			}

			if (lastTime % 100 == 0){
				led1.Switch();
				led0.SetEnable(start);
				if(!start){
					char c[10];
					lcd.SetRegion(Lcd::Rect(0,0,128,15));
					sprintf(c,"FL: %d", raw_front_left);
					writer.WriteBuffer(c,10);
					lcd.SetRegion(Lcd::Rect(0,15,128,15));
					sprintf(c,"FR: %d", raw_front_right);
					writer.WriteBuffer(c,10);
					lcd.SetRegion(Lcd::Rect(0,30,128,15));
					sprintf(c,"ML: %d",raw_mid_left);
					writer.WriteBuffer(c,10);
					lcd.SetRegion(Lcd::Rect(0,45,128,15));
					sprintf(c,"MR: %d",raw_mid_right);
					writer.WriteBuffer(c,10);
					lcd.SetRegion(Lcd::Rect(0,60,128,15));
					sprintf(c,"P: %d %d",equalMax,equalMin);
					writer.WriteBuffer(c,10);
					lcd.SetRegion(Lcd::Rect(0,75,128,15));
					sprintf(c,"Min: %d", oneLineMin);
					writer.WriteBuffer(c,10);
					lcd.SetRegion(Lcd::Rect(0,90,128,15));
					sprintf(c,"A: %d",servo.GetDegree());
					writer.WriteBuffer(c,10);
					lcd.SetRegion(Lcd::Rect(0,120,128,15));
					sprintf(c,"M: %f",frontLinear*100);
					writer.WriteBuffer(c,10);
					lcd.SetRegion(Lcd::Rect(0,135,128,15));
					sprintf(c,"M: %f",midLinear*100);
					writer.WriteBuffer(c, 10);
				}
			}
    	}
    }
    return 0;

}

}  // namespace board1
}  // namespace car1
}  // namespace testing
}  // namespace algorithm


