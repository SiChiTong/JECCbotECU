/*
 * moveUtils.h
 *
 *  Created on: May 12, 2020
 *      Author: jonas
 */

#ifndef INC_MOVEUTILS_H_
#define INC_MOVEUTILS_H_

#include "api.h"

#define MOVEUTILS_SPEEDMAX 0x7fff

extern uint16_t p;
extern int16_t speed;
extern int16_t headingDest;
extern int16_t headingCurrent;
extern int16_t offset;

void moveHeading();

#endif /* INC_MOVEUTILS_H_ */
