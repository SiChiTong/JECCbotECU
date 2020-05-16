/*
 * HTTPServer.h
 *
 *  Created on: May 2, 2020
 *      Author: jonas
 */

#ifndef JECCBOTSERVER_H_
#define JECCBOTSERVER_H_

#include <stdio.h>
#include <sys/socket.h>
#include <unistd.h>
#include <stdlib.h>
#include <netinet/in.h>
#include <string.h>
#include <thread>
#include <vector>
#include <system_error>

#include "Client.h"
#include "RobotCommand.h"

class JECCbotServer {
public:
	JECCbotServer(int port, std::string resourcePath, int maxClients);
	~JECCbotServer();

private:
	int port;
	std::string resourcePath;
	int maxClients;

	int fdServer, fdSocket;

	struct sockaddr_in address;
	int addressLen;

	std::thread *workerThread;
	bool workerThreadStop;
	void worker();

	void startupServer();

	void shutdownServer();

	std::vector<RobotCommand> robotCommandVector;
};
#endif /* JECCBOTSERVER_H_ */
