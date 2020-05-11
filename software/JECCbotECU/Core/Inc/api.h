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

typedef struct ApiInstruction
{
	bool set;
	char command[20];
	int commandLen;
	char response[20];
	int responseLen;
}ApiInstruction;

extern ApiInstruction apiInstruction;

extern uint16_t apiMemory[API_MEMORY_SIZE];
extern bool apiWriteables[API_MEMORY_SIZE];
extern bool apiLocked;

void apiInit();

void apiSetInstruction(char *instruction);

ApiInstruction apiUpdate();

bool isApiAddressValid(int address);

bool apiWrite16(int address, int16_t value);
int16_t apiRead16(int address);

bool apiWrite32(int address, int32_t value);
int32_t apiRead32(int address);

bool apiWriteFloat(int address, float value);
float apiReadFloat(int address);

#endif
