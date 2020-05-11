/*
 * nmeaUtils.h
 *
 *  Created on: May 10, 2020
 *      Author: jonas
 */

#ifndef INC_NMEAUTILS_H_
#define INC_NMEAUTILS_H_

#include "stdint.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "stdbool.h"

#include "apiConfiguration.h"
#include "api.h"

#define NMEA_STR_LEN 200

#define NMEA_HCHDT_LEN 19
#define NMEA_GPRMC_LEN 72

#define NMEA_TIME_OFFSET 2

#define NMEA_HEADING_OFFSET 180

typedef struct NmeaString
{
	char nmeaStr[200];
	bool available;
}NmeaString;

void nmeaDecodeToApi(NmeaString *nmeaString);

void hchdtToApi(NmeaString *nmeaString, int start);
void gprmcToApi(NmeaString *nmeaString, int start);

#endif /* INC_NMEAUTILS_H_ */
