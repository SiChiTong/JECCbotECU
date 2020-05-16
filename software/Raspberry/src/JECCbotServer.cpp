/*
 * HTTPServer.cpp
 *
 *  Created on: May 2, 2020
 *      Author: jonas
 */
#include "JECCbotServer.h"

JECCbotServer::JECCbotServer(int port, std::string resourcePath, int maxClients)
{
	this->port = port;
	this->resourcePath = resourcePath;
	this->maxClients = maxClients;

	printf("Server an Port %d\n", port);

	startupServer();
}

JECCbotServer::~JECCbotServer()
{
	shutdownServer();
}

void JECCbotServer::startupServer()
{
	addressLen = sizeof(address);

	if((fdServer = socket(AF_INET, SOCK_STREAM, 0)) == 0)
	{
		perror("In socket...");
		exit(EXIT_FAILURE);
	}

	address.sin_family = AF_INET;
	address.sin_addr.s_addr = INADDR_ANY;
	address.sin_port = htons(port);

	memset(address.sin_zero, '\0', sizeof(address.sin_zero));

	if(bind(fdServer, (struct sockaddr *)&address, addressLen) < 0)
	{
		perror("In bind...");
		exit(EXIT_FAILURE);
	}

	if(listen(fdServer, maxClients) < 0)
	{
		perror("In listen...");
		exit(EXIT_FAILURE);
	}

	workerThreadStop = false;
	workerThread = new std::thread(&JECCbotServer::worker, this);
	//workerThread->detach();
}

void JECCbotServer::shutdownServer()
{
	workerThreadStop = true;
	workerThread->join();

	close(fdServer);
}

void JECCbotServer::worker()
{
	while(!workerThreadStop)
	{
        if ((fdSocket = accept(fdServer, (struct sockaddr *)&address, (socklen_t*)&addressLen))<0)
        {
            perror("In accept");
            //exit(EXIT_FAILURE);
        }
        else
        {
            Client client(fdSocket, resourcePath, &robotCommandVector);
        }
	}
}

