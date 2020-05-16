/*
 * main.cpp
 *
 *  Created on: May 2, 2020
 *      Author: jonas
 */
#include "JECCbotServer.h"
#include "ECU.h"
#include "DataLogger.h"

int main()
{
	JECCbotServer *server = new JECCbotServer(8000, "/home/pi/resources", 40);
    ECU *ecu = new ECU("/dev/ttyACM0", 115200);
    DataLogger *logger = new DataLogger("/home/pi/resources");

    NmeaBlob n;

	while(1)
	{
        n = ecu->getNmeaData();
        //printf("%i\n%i\n%f\n%f\n", n.heading, n.time, n.latitude, n.longitude);
        logger->logString("nmeaData.txt", "Heading: " + std::to_string(n.heading), true);
        logger->logString("nmeaData.txt", "Time: " + std::to_string(n.time), false);
        logger->logString("nmeaData.txt", "Latitude: " + std::to_string(n.latitude), false);
        logger->logString("nmeaData.txt", "Longitude: " + std::to_string(n.longitude), false);

        usleep(500000);
	}

	return 0;
}
