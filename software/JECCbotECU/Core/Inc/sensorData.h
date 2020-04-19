/*
 * sensorData.h
 *
 *  Created on: Apr 18, 2020
 *      Author: jonas
 */

#ifndef INC_SENSORDATA_H_
#define INC_SENSORDATA_H_

#define LIDAR_BUFFERSIZE 30

typedef struct KvhData
{
	bool available;
	int16_t heading;
}KvhData;

KvhData kvhData;

typedef struct GpsData
{
	bool available;
	uint32_t time;
	float lat;
	float lon;
}GpsData;

GpsData gpsData;

typedef struct LidarData
{
	bool available;
	uint8_t frameBuffer[LIDAR_BUFFERSIZE][5];
}LidarData;

LidarData lidarData;

#endif /* INC_SENSORDATA_H_ */
