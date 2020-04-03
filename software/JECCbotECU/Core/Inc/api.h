/*
 * api.h
 *
 *  Created on: Mar 26, 2020
 *      Author: jonas
 */

#ifndef INC_API_H_
#define INC_API_H_

#include "stdint.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "stdbool.h"

#include "apiConfiguration.h"

extern uint16_t apiMemory[API_MEMORY_SIZE];
extern bool apiWriteables[API_MEMORY_SIZE];
extern bool apiLocked;

void apiInit();

void apiDoInstruction(char* instruction, char* responseMessage);

#endif /* INC_API_H_ */
