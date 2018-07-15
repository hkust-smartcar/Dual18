/*
 * variable.h
 *
 *  Created on: Jul 15, 2018
 *      Author: PWS
 */

#ifndef INC_VARIABLE_H_
#define INC_VARIABLE_H_

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

#endif /* INC_VARIABLE_H_ */
