/*
 * HTTPDecoder.h
 *
 *  Created on: May 2, 2020
 *      Author: jonas
 */

/*
 * HTTPDecoder.h
 *
 *  Created on: Dec 31, 2019
 *      Author: jonas
 */

#ifndef HTTPDECODER_H_
#define HTTPDECODER_H_
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <fstream>
#include <iostream>
#include <sstream>
#include <fcntl.h>
#include <unistd.h>
#include <algorithm>
#include <iterator>
#include <time.h>

#define HTTP_REQUEST_TYPE_GET 0
#define HTTP_REQUEST_TYPE_POST 1

typedef struct Request
{
	int type;
	std::string url;

	std::string postMessage;
}Request;

typedef struct Response
{
    char *content;
    int len;
}Response;

class HTTPDecoder {
public:
	HTTPDecoder(std::string resourcePath);
	~HTTPDecoder();

	Request decode(char *request);

    Response generateResponse(Request req);

private:

    static std::string favicon;

    std::string resourcePath;

};

#endif /* HTTPDECODER_H_ */
