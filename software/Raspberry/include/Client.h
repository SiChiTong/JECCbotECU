/*
 * Client.h
 *
 *  Created on: May 2, 2020
 *      Author: jonas
 */

#ifndef CLIENT_H_
#define CLIENT_H_

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <string.h>
#include <thread>
#include <vector>
#include <mutex>
#include <system_error>

#include "RobotCommand.h"
#include "HTTPDecoder.h"

class Client
{
public:
	Client(int fdSocket, std::string resourcePath, std::vector<RobotCommand> *commandVector);
	~Client();

private:
	int fdSocket;
	std::vector<RobotCommand> *commandVector;

	std::thread *workerThread;
	void worker();

    std::mutex mu;

	HTTPDecoder *dec;
};

#endif /* CLIENT_H_ */
