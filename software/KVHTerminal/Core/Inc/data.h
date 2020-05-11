/*
 * data.h
 *
 *  Created on: May 11, 2020
 *      Author: jonas
 */

#ifndef INC_DATA_H_
#define INC_DATA_H_

#include <stdbool.h>
#include <stdint.h>

typedef struct USBData
{
	bool available;
	char data[100];
	uint32_t len;
}USBData;

extern USBData usbData;

#endif /* INC_DATA_H_ */
