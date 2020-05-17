/*
 * geoUtils.c
 *
 *  Created on: May 14, 2020
 *      Author: jonas
 */

#ifndef SRC_GEOUTILS_C_
#define SRC_GEOUTILS_C_

#include "geoUtils.h"

void geoDistanceAndHeadingToApi()
{
	float la1 = apiReadFloat(API_BENCH_GPS_START + 2);
	float lo1 = apiReadFloat(API_BENCH_GPS_START + 4);
	float la2 = apiReadFloat(API_BENCH_DESTCOORDS_START);
	float lo2 = apiReadFloat(API_BENCH_DESTCOORDS_START + 2);

	float latitude1R = (la1*M_PI)/180.0;
    float longitude1R = (lo1*M_PI)/180.0;
    float latitude2R = (la2*M_PI)/180.0;
    float longitude2R = (lo2*M_PI)/180.0;

    float dlat = (latitude2R - latitude1R);
    float dlong = (longitude2R - longitude1R);
    float a = sin(dlat/2.0) * sin(dlat/2.0) +
            cos(latitude1R) * cos(latitude2R) *
            sin(dlong/2.0) * sin(dlong/2.0);
    int16_t distance = (int16_t)(6371000* 2 * atan2(sqrt(a), sqrt(1-a)));
    apiMemory[API_REG_DEST_DIST] = distance;

    float y = sin(longitude2R-longitude1R) * cos(latitude2R);
    float x = cos(latitude1R)*sin(latitude2R) -
            sin(latitude1R)*cos(latitude2R)*cos(longitude2R-longitude1R);
    int16_t heading = (int16_t)((atan2(y, x)/(2*M_PI))*360.0);
    apiMemory[API_REG_MOVEHEADING_HEADING] = heading;
}

#endif /* SRC_GEOUTILS_C_ */
