/*
 * variable.h
 *
 *  Created on: Jul 15, 2018
 *      Author: PWS
 */

#ifndef INC_VARIABLE_H_
#define INC_VARIABLE_H_

#include "PID.h"

static PID servoPIDx(0, 0);
static PID servoPIDy(0, 0);
static PID servoPIDAlignCurve(0, 0);
static float x_servo_pd[2],y_servo_pd[2],align_servo_pd[2];

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
static carState magState = kNormal;

static const uint8_t cycle = 12;
static float loopSpeed = 9, highSpeed = 9, alignSpeed = 9;
static float speed = highSpeed;
static bool approaching = false, isFirst = false, firstArrived = false, secondArrived = false, USsent = false;
static bool left_loop = false;
static uint8_t leaveCount = 0;
static uint32_t lastTime = 0, approachTime = 0;
static float angle = 0, angleX = 0, angleY = 0;
static float l1 = 100,l2 = 100,r1 = 100,r2 = 100;

#endif /* INC_VARIABLE_H_ */
