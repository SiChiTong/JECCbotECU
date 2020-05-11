/*
 * nmeaUtils.c
 *
 *  Created on: May 10, 2020
 *      Author: jonas
 */
#include "nmeaUtils.h"

void nmeaDecodeToApi(NmeaString *nmeaString)
{
	if(strncmp("$HCHDT", nmeaString->nmeaStr, 6) == 0)
	{
		hchdtToApi(nmeaString);
	}
	else if(strncmp("$GNRMC", nmeaString->nmeaStr, 6) == 0 || strncmp("$GPRMC", nmeaString->nmeaStr, 6) == 0)
	{
		rmcToApi(nmeaString);
	}
}

void hchdtToApi(NmeaString *nmeaString)
{
	char headingStr[4];
	strncpy(headingStr, &nmeaString->nmeaStr[7], 3);
	headingStr[3] = '\0';

	int heading = strtol(headingStr, NULL, 10);
	if(heading > 180)
	{
		heading -= 360;
	}

	apiWrite16(API_REG_HEADING_KVH, heading);
}

void rmcToApi(NmeaString *nmeaString)
{
	int fieldIndex=0;
	int charIndex=0;

	char fields[13][15];
	char currentField[15];

	float lat, lon;

	for(int i=7; i<strlen(nmeaString->nmeaStr); i++)
	{
	  char currentChar=nmeaString->nmeaStr[i];
	  if(currentChar!=',')
	  {
	    currentField[charIndex]=currentChar;
	    charIndex++;
	  }
	  else
	  {
		currentField[charIndex] = '\0';
	    strcpy(fields[fieldIndex], currentField);
	    charIndex = 0;
	    fieldIndex++;
	    if(fieldIndex > 5)
	    {
	    	break;
	    }
	  }
	}

	//decode time -> fieldIndex 0
	char timeStr[7];
	uint32_t time;
	strncpy(timeStr, fields[0], 6);
	timeStr[6] = '\0';
	time = strtol(timeStr, NULL, 10);
	time += NMEA_TIME_OFFSET * 10000;
	if(time > 240000)
	{
		time -= 240000;
	}

		//decode latitude -> fieldIndex 2/3
		char ddLat[3];
		char mmLat[8];
		strncpy(ddLat, fields[2], 2);
		ddLat[2] = '\0';
		strncpy(mmLat, &fields[2][2], 7);
		mmLat[7] = '\0';

		lat=atof(ddLat) + atof(mmLat)/60;
		if(fields[3][0]=='S')
		  lat=-lat;


//			//decode longitude -> fieldIndex 4/5
		char ddLon[4];
		char mmLon[8];
		strncpy(ddLon, fields[4], 3);
		ddLon[3] = '\0';
		strncpy(mmLon, &fields[4][3], 7);
		mmLon[7] = '\0';

		lon=atof(ddLon) + atof(mmLon)/60;
		if(fields[6][0]=='W')
		  lon=-lon;

		apiWrite32(API_BENCH_GPS_START, time);
		apiWriteFloat(API_BENCH_GPS_START + 2, lat);
		apiWriteFloat(API_BENCH_GPS_START + 4,  lon);
}
