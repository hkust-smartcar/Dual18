/*
 * joseph_dual_cam.h
 *
 *  Created on: Apr 16, 2018
 *      Author: morristseng
 */

#ifndef INC_JOSEPH_DUAL_CAM_H_
#define INC_JOSEPH_DUAL_CAM_H_

/* Dual cam smaple program
 * @brief:
 * The program outputs two camera images on the LCD
 *
 * Hardare pins config: VCAN_FX15DEV
 *
 *
 #define LIBSC_USE_BUTTON 1
 #define LIBSC_USE_LED 4
 #define LIBSC_USE_UART 1
 #define LIBSC_USE_OV7725 2
 #define LIBSC_USE_LCD 1

 #define LIBSC_BUTTON0 libbase::k60::Pin::Name::kPtd7

 #define LIBSC_LED0 libbase::k60::Pin::Name::kPtb20
 #define LIBSC_LED1 libbase::k60::Pin::Name::kPtb21
 #define LIBSC_LED2 libbase::k60::Pin::Name::kPtb22
 #define LIBSC_LED3 libbase::k60::Pin::Name::kPtb23

 #define LIBSC_UART0_TX libbase::k60::Pin::Name::kPtb17
 #define LIBSC_UART0_RX libbase::k60::Pin::Name::kPtb16

 #define LIBSC_OV77250_VSYNC libbase::k60::Pin::Name::kPtc11
 #define LIBSC_OV77250_PCLK libbase::k60::Pin::Name::kPtc12
 #define LIBSC_OV77250_SDA libbase::k60::Pin::Name::kPtc2
 #define LIBSC_OV77250_SCL libbase::k60::Pin::Name::kPtc1
 #define LIBSC_OV77250_DATA0 libbase::k60::Pin::Name::kPtc3
 #define LIBSC_OV77250_DMA_CH 4

 #define LIBSC_OV77251_VSYNC libbase::k60::Pin::Name::kPta6
 #define LIBSC_OV77251_PCLK libbase::k60::Pin::Name::kPta7
 #define LIBSC_OV77251_SDA libbase::k60::Pin::Name::kPta17
 #define LIBSC_OV77251_SCL libbase::k60::Pin::Name::kPta16
 #define LIBSC_OV77251_DATA0 libbase::k60::Pin::Name::kPta8
 #define LIBSC_OV77251_DMA_CH 6

 #define LIBSC_ST7735R_DC libbase::k60::Pin::Name::kPte0
 #define LIBSC_ST7735R_SDAT libbase::k60::Pin::Name::kPte1
 #define LIBSC_ST7735R_SCLK libbase::k60::Pin::Name::kPte2
 #define LIBSC_ST7735R_RST libbase::k60::Pin::Name::kPte3
 #define LIBSC_ST7735R_CS libbase::k60::Pin::Name::kPte4

 */

#include <cmath>
#include <vector>
#include <cassert>
#include <cstring>
#include <string>

#include <libbase/k60/mcg.h>
#include <libsc/system.h>
#include <libbase/k60/uart.h>
#include "libsc/led.h"
#include "libsc/button.h"
#include "libsc/lcd_typewriter.h"
#include "libsc/st7735r.h"
#include "libsc/k60/ov7725.h"

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
using libsc::k60::Ov7725;

Led::Config GetLedConfig(uint8_t id) {
	Led::Config config;
	config.id = id;
	return config;
}

LcdTypewriter::Config GetWriterConfig(St7735r *lcd) {
	LcdTypewriter::Config config;
	config.lcd = lcd;
	return config;
}

St7735r::Config GetLcdConfig() {
	//TODO: finish it
	St7735r::Config config;
	config.fps = 20;
	return config;
}

Ov7725::Config getCameraConfig(uint8_t id) {
	Ov7725::Config config;
	config.id = id;
	config.w = 80;
	config.h = 60;
	config.fps = Ov7725::Config::Fps::kHigh;
	return config;
}

int main() {
	System::Init();

	uint32_t lastTime = 0;
	int timeDif = 0;

	Ov7725 camera0(getCameraConfig(0));
	camera0.Start();

	Ov7725 camera1(getCameraConfig(1));
	camera1.Start();

	Led led(GetLedConfig(0));
	St7735r lcd(GetLcdConfig());
	LcdTypewriter writer(GetWriterConfig(&lcd));

	bool t = false;

	uint32_t TimeIn125US = System::TimeIn125us();

	while (true) {
		if (System::Time() != lastTime) {
			lastTime = System::Time();
			if (lastTime % 100 == 0) {
				led.Switch();

//				 cam 0
				const Byte* Buffer0 = camera0.LockBuffer();
				camera0.UnlockBuffer();

				lcd.SetRegion(Lcd::Rect(0, 0, 80, 60));
				lcd.FillBits(Lcd::kBlack, Lcd::kWhite, Buffer0, 60 * 80);

//				 cam 1
				const Byte* Buffer1 = camera1.LockBuffer();
				camera1.UnlockBuffer();

				lcd.SetRegion(Lcd::Rect(0, 60, 80, 60));
				lcd.FillBits(Lcd::kBlack, Lcd::kWhite, Buffer1, 60 * 80);

//				 time
				char c[14] = "             ";
				timeDif = System::Time() - lastTime;
				sprintf(c, "Time: %d", timeDif);
				lcd.SetRegion(Lcd::Rect(0, 120, 80, 15));
				writer.WriteBuffer(c, 14);

//				 flash
				lcd.SetRegion(Lcd::Rect(0, 135, 80, 30));
				if (t) {
					lcd.FillColor(Lcd::kGreen);
				} else {
					lcd.FillColor(Lcd::kBlue);
				}

				t = !t;
			}
		}
	}
}



#endif /* INC_JOSEPH_DUAL_CAM_H_ */
