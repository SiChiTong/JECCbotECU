/*
 * Client.cpp
 *
 *  Created on: May 2, 2020
 *      Author: jonas
 */
#include "Client.h"

Client::Client(int fdSocket, std::string resourcePath, std::vector<RobotCommand> *commandVector)
{
	this->fdSocket = fdSocket;
	this->commandVector = commandVector;

	dec = new HTTPDecoder(resourcePath);

	workerThread = new std::thread(&Client::worker, this);

	//workerThread->detach();
}

Client::~Client()
{
	//workerThread->join();
}

void Client::worker()
{
	char buffer[30000] = {0};
//	printf("%d\n", fdSocket);
	int valread = read(fdSocket, buffer, 30000);
//	printf("%s\n", buffer);
    Request req = dec->decode(buffer);

    Response rep = dec->generateResponse(req);
    write(fdSocket, rep.content, rep.len);

    if(HTTP_REQUEST_TYPE_POST == req.type)
    {
    	RobotCommand command;
    	command.rawMessage = req.postMessage;
    	mu.lock();
    	commandVector->push_back(command);
    	mu.unlock();
    }
	close(fdSocket);
	//workerThread->join();
}

