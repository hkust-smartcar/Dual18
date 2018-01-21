/*
 * main.cpp
 *
 * Author: Amrutavarsh S Kinagi, Morris Tseng
 * Copyright (c) 2014-2015 HKUST SmartCar Team
 * Refer to LICENSE for details
 */

#include <cassert>
#include <cstring>
#include <functional>
#include <libbase/k60/mcg.h>
#include <libsc/system.h>
#include <libsc/led.h>
#include <libsc/lcd_typewriter.h>
#include <libbase/k60/gpio.h>
#include <libsc/st7735r.h>
#include <libsc/lcd.h>
#include <libsc/k60/ov7725.h>
#include <libsc/k60/ov7725_configurator.h>
#include <libsc/lcd_typewriter.h>
#include <libsc/futaba_s3010.h>
#include <vector>
#include <math.h>
#include <libsc/alternate_motor.h>
#include <libbase/k60/adc.h>
#include <libsc/joystick.h>

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
using namespace std;

#define Width 80
#define Height 60
bool camptr[Height][Width];

inline bool ret_cam_bit(int x, int y, const Byte* camBuffer) {
    return ((camBuffer[y * 10 + x / 8] >> (7 - (x % 8))) & 1);
}

St7735r* lcdP;

St7735r::Config getLcdConfig() {
    St7735r::Config config;
    config.orientation = 2;
    return config;
}

LcdTypewriter::Config GetWriterConfig(St7735r *temp_lcd) {
    LcdTypewriter::Config config;
    config.lcd = temp_lcd;
    config.bg_color = 0;
    config.text_color = 0xFFFF;
    config.is_text_wrap = false;
    return config;
}

k60::Ov7725::Config getCameraConfig() {
    k60::Ov7725::Config config;
    config.id = 0;
    config.w = Width;
    config.h = Height;
    config.fps = k60::Ov7725Configurator::Config::Fps::kHigh;
    return config;
}

FutabaS3010::Config getServoConfig() {
    FutabaS3010::Config config;
    config.id = 0;
    return config;
}

AlternateMotor::Config getMotorConfig() {
    AlternateMotor::Config config;
    config.id = 0;
    return config;
}

Adc::Config getMagSensorConfig(Pin::Name pin_name) {
    Adc::Config config;
    config.pin = pin_name;
    config.speed = Adc::Config::SpeedMode::kFast;
    config.is_continuous_mode = true;
    config.avg_pass = Adc::Config::AveragePass::k32;
    return config;
}

Joystick::Config getJoystickConfig(Joystick::Listener isr) {
    Joystick::Config config;
    config.id = 0;
    config.is_active_low = true;
    config.dispatcher = isr;
    return config;
}
Led::Config getLedConfig(int led_id){
	Led::Config config;
	config.id = led_id;
	return config;
}

struct Bits{
    int x, y;
    bool is_count_before;
};

int is_equal(Bits first, Bits second){
    if((first.x == second.x)&&(first.y == second.y)&&(first.is_count_before == second.is_count_before)){
        return true;
    }
    else{
        return false;
    }
}

int right_count =0;
bool see_the_right_curve = false;
bool see_the_right_cycle_or_not(int top_line, int bottom_line, const Byte* camBuffer){
	right_count = 0;
	see_the_right_curve = false;
	int do_not_need_to_count =0;
	vector<Bits> left_my_bit;
	vector<Bits> right_my_bit;
	bool is_x_edge = false;
	bool is_y_edge = false ;
	bool is_both_edge = false;
	for (int y=top_line+2; y<top_line+3; y++){
		for (int x=40 ; x<80; x++){
			for(int j=77; j<Width; j++){
				for(int i=top_line; i<top_line+2 ; i++){
					if(ret_cam_bit(j, i, camBuffer)==1){
						is_both_edge = true;
						break;
					}
				}
				if(is_both_edge == true){
					break;
				}
				for(int i = bottom_line; i>bottom_line-2 ; i--){
					if(ret_cam_bit(j, i, camBuffer)==1){
						is_both_edge = true;
						break;
					}
				}
				if(is_both_edge == true){
					break;
				}
				for(int i = bottom_line; i>bottom_line-2 ; i--){
					if((ret_cam_bit(j-39, i, camBuffer)==1)){
						is_both_edge = true;
						break;
					}
				}
				if(is_both_edge == true){
					break;
				}
			}
			if(is_both_edge==false){
				for (int y=top_line; y<top_line+3; y++){
					for (int x=40; x<80; x++){
						if ((ret_cam_bit(x, y, camBuffer) != ret_cam_bit(x-1, y, camBuffer))||(ret_cam_bit(x, y, camBuffer) != ret_cam_bit(x, y-1, camBuffer))){
							is_x_edge = true;
						}
					}
				}
			}

			for (int y=top_line; y<bottom_line; y++){
				for (int x=77 ; x<80; x++){
					if ((ret_cam_bit(x, y, camBuffer) != ret_cam_bit(x-1, y, camBuffer))||(ret_cam_bit(x, y, camBuffer) != ret_cam_bit(x, y-1, camBuffer))){
						is_y_edge = true;
						break;
					}
				}
				if(is_y_edge == true){
					break;
				}
			}

			if((is_x_edge==true)&&(is_y_edge==true)){
				for(int j=top_line;j<bottom_line;j++){
					for(int i=40;i<80;i++){
						if(ret_cam_bit(i, j, camBuffer) != 1){
							Bits mybit;
							mybit.x = i;
							mybit.y = j;
							mybit.is_count_before = true;
							right_my_bit.push_back(mybit);
							right_count++;
							}
						else {break;}
					}
				}

				for(int i=40;i<80;i++){
					for(int j=bottom_line-1;j>=top_line;j--){
						if(ret_cam_bit(i, j, camBuffer) != 1){
						Bits mybit2;
						mybit2.x =i;
						mybit2.y =j;
						mybit2.is_count_before = true;
						for (int i=0; i<right_my_bit.size(); i++){
							if (is_equal(mybit2,right_my_bit[i])){
								do_not_need_to_count++;
								break;
							}
						}
							right_count++;
						}
						else {break;}
					}
				}
				right_count = right_count-do_not_need_to_count;
				do_not_need_to_count = 0;


				for(int j=top_line;j<bottom_line;j++){
					for(int i=40;i<80;i++){
						if(ret_cam_bit(i, j, camBuffer) == 1){
							right_count++;
						}
					}
				}

				bool see_the_right_corner = false;
				if((right_count<340)&&(right_count>300)){
					see_the_right_curve = true;
				}
				if(right_count<280){
					see_the_right_corner = true;
				}
				if(see_the_right_corner == true){
					return true;
				}
				else{
					return false;
				}
			}
			else{
				return false;
			}
		}
	}
}

int left_count =0;
bool see_the_left_curve = false;
bool see_the_left_cycle_or_not(int top_line, int bottom_line, const Byte* camBuffer){
	left_count = 0;
	see_the_left_curve = false;
	int do_not_need_to_count =0;
	vector<Bits> left_my_bit;
	vector<Bits> right_my_bit;
	int is_both_edge = false;
	bool is_x_edge = false, is_y_edge = false ;
//	St7735r mylcd(getLcdConfig());
//	St7735r* mylcdP = &mylcd;
//	LcdTypewriter writer(GetWriterConfig(mylcdP));
	for (int y=top_line+2; y<top_line+3; y++){
		for (int x=1 ; x<40; x++){
			for(int j=0; j<3; j++){
				for(int i=top_line; i<top_line+2 ; i++){
					if(ret_cam_bit(j, i, camBuffer)==1){
						is_both_edge = true;
						break;
					}
				}
				if(is_both_edge == true){
					break;
				}
				for(int i = bottom_line; i>bottom_line-2 ; i--){
					if(ret_cam_bit(j, i, camBuffer)==1){
						is_both_edge = true;
						break;
					}
				}
				if(is_both_edge == true){
					break;
				}
				for(int i = bottom_line; i>bottom_line-2 ; i--){
					if((ret_cam_bit(j+38, i, camBuffer)==1)){
						is_both_edge = true;
						break;
					}
				}
				if(is_both_edge == true){
					break;
				}
			}
			if(is_both_edge==false){
				for (int y=top_line; y<top_line+3; y++){
					for(int x=1; x<40; x++){
						if ((ret_cam_bit(x, y, camBuffer) != ret_cam_bit(x-1, y, camBuffer))||(ret_cam_bit(x, y, camBuffer) != ret_cam_bit(x, y-1, camBuffer))){
							is_x_edge = true;
						}
					}
				}
			}
			for (int y=top_line; y<bottom_line; y++){
				for (int x=1 ; x<4; x++){
					if ((ret_cam_bit(x, y, camBuffer) != ret_cam_bit(x-1, y, camBuffer))||(ret_cam_bit(x, y, camBuffer) != ret_cam_bit(x, y-1, camBuffer))){
						is_y_edge = true;
						break;
					}
				}
				if(is_y_edge == true){
					break;
				}
			}
			if((is_x_edge==true)&&(is_y_edge==true)){
				for(int j=top_line;j<bottom_line;j++){
					for(int i=40;i>0; i--){
						if(ret_cam_bit(i, j, camBuffer) != 1){
							Bits mybit;
							mybit.x = i;
							mybit.y = j;
							mybit.is_count_before = true;
							left_my_bit.push_back(mybit);
							left_count++;
				//							lcdP->SetRegion(Lcd::Rect(i,j,1,1));
				//							lcdP->FillColor(Lcd::kBlack);
							}
						else {break;}
					}
				}

				for(int i=40;i>0;i--){
					for(int j=bottom_line-1;j>=top_line;j--){
						if(ret_cam_bit(i, j, camBuffer) != 1){
				//							lcdP->SetRegion(Lcd::Rect(i,j,1,1));
				//							lcdP->FillColor(Lcd::kBlack);
							Bits mybit2;
							mybit2.x =i;
							mybit2.y =j;
							mybit2.is_count_before = true;
							for (int i=0; i<left_my_bit.size(); i++){
								if (is_equal(mybit2,left_my_bit[i])){
									do_not_need_to_count++;
									break;
								}
							}
							left_count++;
						}
						else {break;}
					}
				}
				left_count = left_count-do_not_need_to_count;
				do_not_need_to_count = 0;


				for(int j=top_line;j<bottom_line;j++){
					for(int i=40;i>0;i--){
						if(ret_cam_bit(i, j, camBuffer) == 1){
							left_count++;
						}
					}
				}

				bool see_the_left_corner = false;
				if((left_count<340)&&(left_count>200)){
					see_the_left_curve = true;
				}
				if(left_count<280){
					see_the_left_corner = true;
				}
				if(see_the_left_corner == true){
					return true;
				}
				else{
					return false;
				}
			}
			else{
				return false;
			}
		}
	}
}

bool finish_line(const Byte* camBuffer){
	int row_state_change = 0;
	int avg_state_change = 0;
	for(int j = 30;j<34;j++){
		row_state_change = 0;
		for(int i=1;i<80;i++){
			if(ret_cam_bit(i,j,camBuffer) != ret_cam_bit(i-1,j,camBuffer))
				row_state_change++;
		}
		avg_state_change += row_state_change/4;
	}
	if(avg_state_change > 8)
		return true;
	else return false;
}
//bool isRight(int top_line, int bottom_line,const Byte* camBuffer){
//    int top_exist = 0;
//    int right_exist = 0;
//    for(int i=45;i<75;i++)
//        if(ret_cam_bit(i,top_line,camBuffer) == 1) top_exist++;
//    for(int i=top_line+1;i<bottom_line;i++)
//        if(ret_cam_bit(79,i,camBuffer) == 1) right_exist++;
//    if(top_exist>0 && top_exist<4 && right_exist>0 && right_exist<4)
//        return true;
//    else
//        return false;
//}
//
//bool isLeft(int top_line, int bottom_line,const Byte* camBuffer){
//    int top_exist = 0;
//    int left_exist = 0;
//    for(int i=5;i<35;i++)
//        if(ret_cam_bit(i,top_line,camBuffer) == 1) top_exist++;
//    for(int i=top_line+1;i<bottom_line;i++)
//        if(ret_cam_bit(0,i,camBuffer) == 1) left_exist++;
//    if(top_exist>0 && top_exist<4 && left_exist>0 && left_exist<4)
//        return true;
//    else
//        return false;
//}

int main(void) {
    System::Init();

    int32_t ticks = System::Time();
    int pre_temp = 0;
    int temp = 0;
    float derivative = 0;
    int top_line = 20;
    int bottom_line = 30;
    int Tstate = 0;
    int Lstate = 0;
    bool left_corner = 0;
    bool right_corner = 0;
	Led led1(getLedConfig(0));
	Led led2(getLedConfig(1));

    enum track_state{
        regular = 0,
        right_loop,
        left_loop,
        magnetic,
        finish
    };

    enum loop_state{
        start = 0,
        mid,
        end
    };

    k60::Ov7725 camera(getCameraConfig());
    camera.Start();

    St7735r lcd(getLcdConfig());
    lcdP = &lcd;

    LcdTypewriter writer(GetWriterConfig(&lcd));

    FutabaS3010 Servo(getServoConfig());

    AlternateMotor Motor(getMotorConfig());
    Motor.SetPower(155);
    Motor.SetClockwise(false);

    Adc MagLeft(getMagSensorConfig(Pin::Name::kPtb1));
    Adc MagRight(getMagSensorConfig(Pin::Name::kPtc10));


    while (true) {
        while (ticks != libsc::System::Time()) {
            ticks = libsc::System::Time();
            if (ticks % 10 == 0) {
                ticks = System::Time();

                Motor.SetPower(165);
                char c[15];
                see_the_left_curve = false;
                see_the_right_curve = false;

                const Byte* camBuffer = camera.LockBuffer();
//                lcdP->SetRegion(Lcd::Rect(0, 0, Width, Height));
//                lcdP->FillBits(0x0000, 0xFFFF, camBuffer, Width * Height);
//                lcdP->SetRegion(Lcd::Rect(0,20 , Width, 2));
//                lcdP->FillColor(Lcd::kRed);
//                lcdP->SetRegion(Lcd::Rect(0,30 , Width, 2));
//                lcdP->FillColor(Lcd::kBlue);
                camera.UnlockBuffer();
                int l_count = 0;
                int r_count = 0;
                int l_large = 0;
                int r_large = 0;
                int l_small = 0;
                int r_small = 0;
                for (int j = 45; j < 50; j++) {
                    int l_counter = 0;
                    int r_counter = 0;
                    for (int i = 39; i >= 0 && ret_cam_bit(i, j, camBuffer) != 1;i--)
                        l_counter++;
                    for (int k = 40; k < 80 && ret_cam_bit(k, j, camBuffer) != 1; k++)
                        r_counter++;
                    l_count += l_counter;
                    r_count += r_counter;
                    if (l_counter > 37)
                        l_large++;
                    if (r_counter > 37)
                        r_large++;
                    if (l_counter < 4)
                        l_small++;
                    if (r_counter < 4)
                        r_small++;
                }

                temp = ((r_count - l_count) * 90) / (r_count + l_count);
                if ((l_large && r_large) || l_small || r_small)
                    temp = pre_temp;
                derivative = (-temp + pre_temp) / 10;

                //Multiple track states
                if(Tstate == regular){
//					if(isRight(top_line,bottom_line,camBuffer))
//                    right_corner = see_the_right_cycle_or_not(top_line, bottom_line, camBuffer);
//					else
//						right_corner = false;
//					if(isLeft(top_line,bottom_line,camBuffer))
//                    left_corner = see_the_left_cycle_or_not(top_line, bottom_line, camBuffer);
//					else
//						left_corner = false;
					right_corner = see_the_right_cycle_or_not(top_line, bottom_line, camBuffer);
					left_corner = see_the_left_cycle_or_not(top_line, bottom_line, camBuffer);
                    if(finish_line(camBuffer))
                    		Tstate = finish;
                    else if(right_corner && !left_corner){
                        Tstate = right_loop;
                        led1.Switch();
                    }
                    else if(!right_corner && left_corner)
                        Tstate = left_loop;
                    else if(MagRight.GetResult()>17 || MagRight.GetResult()>17)
                        Tstate = magnetic;
                    else
                        Tstate = regular;
                }
                else if(Tstate == right_loop){
                    if(Lstate == loop_state::start){
                        if(!l_large){
                            temp = 50;
                            derivative = 0;
                        }
                        else if(l_large) {
                            temp = 50;
                            derivative = 0;
                            Lstate = loop_state::mid;
                        }
                    }
                    else if(Lstate == loop_state::mid){
                        if(!l_large) Lstate = loop_state::end;
                    }
                    else if(Lstate == loop_state::end){
                        if(l_large){
                            Tstate = regular;
                            Lstate = loop_state::start;
                            temp = 45;
                            derivative = 0;
                        }
                    }
                }
                else if(Tstate == left_loop){
                    if(Lstate == loop_state::start){
                        if(!r_large){
                            temp = -50;
                            derivative = 0;

                        }
                        else if(r_large) {
                            temp = -50;
                            derivative = 0;
                            Lstate = loop_state::mid;
                        }
                    }
                    else if(Lstate == loop_state::mid){
                        if(!r_large) Lstate = loop_state::end;
                    }
                    else if(Lstate == loop_state::end){
                        if(r_large){
                            Tstate = regular;
                            Lstate = loop_state::start;
                            temp = -45;
                            derivative = 0;
                        }
                    }
                }
                else if(Tstate == magnetic){
                		if(finish_line(camBuffer))
                	        Tstate = finish;
                		else if(MagRight.GetResult()<16 && MagLeft.GetResult()<16){
                        Tstate = regular;
                    }
                    temp = ((MagRight.GetResult() - MagLeft.GetResult()) * 100) / (MagRight.GetResult() + MagLeft.GetResult());
                    derivative = (-temp + pre_temp) / 10;
                }
                else if(Tstate == finish){
                		Servo.SetDegree(900);
                		Motor.SetPower(10);
                		System::DelayS(1);
                		Motor.SetPower(0);
                		break;
                }

                //Setting servo angle with multiple PD and motor values
                if(see_the_right_curve || see_the_left_curve){
					Motor.SetPower(0);
					Servo.SetDegree(900 - (1.25 * (-temp) + 0 * derivative) * 10);
					led2.Switch();
                	}
                else if(left_corner && right_corner){
					Servo.SetDegree(900 - (1.45 * (-temp) + 0 * derivative) * 10);
					Motor.SetPower(0);
				}
				else if(Lstate == loop_state::start && (Tstate == left_loop || Tstate == right_loop)){
					Servo.SetDegree(900 - (1.45 * (-temp) + 0 * derivative) * 10);
					Motor.SetPower(0);
				}
				else if (((l_large && r_large) || l_small || r_small) && Tstate!=magnetic){
					Servo.SetDegree(900 - (1.45 * (-temp) + 0 * derivative) * 10);
					Motor.SetPower(120);
				}
                else if(temp<25 && temp>-20){
                    Servo.SetDegree(900 - (0.825 * (-temp) + 0.25 * derivative) * 10);
                }
                else if(temp<40 && temp>-40){
                    Servo.SetDegree(900 - (1.25 * (-temp) + 0 * derivative) * 10);
					Motor.SetPower(120);
                }
                else{
					Servo.SetDegree(900 - (1.405 * (-temp) + 0 * derivative) * 10);
					Motor.SetPower(130);
                }

                pre_temp = temp;


//                lcdP->SetRegion(Lcd::Rect(0,79,120,15));
//				sprintf(c,"left :%d   ",left_count);
//				writer.WriteBuffer(c,15);
//				lcdP->SetRegion(Lcd::Rect(0,96,120,15));
//				sprintf(c,"right :%d   ",right_count);
//				writer.WriteBuffer(c,15);
//                lcdP->SetRegion(Lcd::Rect(0,62,120,15));
//                sprintf(c,"Track :%d",Tstate);
//                writer.WriteBuffer(c,15);
//                lcdP->SetRegion(Lcd::Rect(0,79,120,15));
//                sprintf(c,"Loop :%d",Lstate);
//                writer.WriteBuffer(c,15);
//                lcdP->SetRegion(Lcd::Rect(0,96,120,15));
//                sprintf(c,"Right :%d",right_corner);
//                writer.WriteBuffer(c,15);
//                lcdP->SetRegion(Lcd::Rect(0,113,120,15));
//                sprintf(c,"Left :%d",left_corner);
//                writer.WriteBuffer(c,15);
//				lcdP->SetRegion(Lcd::Rect(0,130,100, 15));
//				sprintf(c, "Rcount: %d", right_corner_value);
//				writer.WriteBuffer(c, 15);
//				lcdP->SetRegion(Lcd::Rect(0,147,100, 15));
//				sprintf(c, "Lcount: %d", left_corner_value);
//				writer.WriteBuffer(c, 15);
            }
        }
    }

    return 0;
}
