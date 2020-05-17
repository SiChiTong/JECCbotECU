/*
 * moveUtils.c
 *
 *  Created on: May 12, 2020
 *      Author: jonas
 */
#include "moveUtils.h"

uint16_t p;
int16_t speed;
int16_t headingDest;
int16_t headingCurrent;
int16_t offset;

void moveHeading()
{


	p = apiMemory[API_REG_MOVEHEADING_P];
	speed = apiMemory[API_REG_MOVEHEADING_SPEED];
	headingDest = apiMemory[API_REG_MOVEHEADING_HEADING];
	headingCurrent = apiMemory[API_REG_HEADING_KVH];

	offset = headingDest - headingCurrent;

	if(offset>180)
	{
	  offset=offset-360;
	}
	else if(offset<-180)
	{
	  offset=360+offset;
	}

	int offsetSpeed = offset * p;

	int left = speed + offsetSpeed;
	int right = speed - offsetSpeed;

	  if(left<0)
	    left=0;
	  else if(left>=MOVEUTILS_SPEEDMAX)
	    left=MOVEUTILS_SPEEDMAX;

	  if(right<0)
	    right=0;
	  else if(right>=MOVEUTILS_SPEEDMAX)
	    right=MOVEUTILS_SPEEDMAX;

	  apiWrite16(API_REG_PWMLEFT, left);
	  apiWrite16(API_REG_PWMRIGHT, right);
}

//void moveGPS()
//{
//	  geoDistanceAndHeadingToApi();
//	  moveHeading();
//}
