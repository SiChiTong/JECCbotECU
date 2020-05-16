/*
 * HTTPDecoder.cpp
 *
 *  Created on: May 2, 2020
 *      Author: jonas
 */

#include "HTTPDecoder.h"

std::string HTTPDecoder::favicon = "index.html";

HTTPDecoder::HTTPDecoder(std::string resourcePath) {
	this->resourcePath = resourcePath;
}

HTTPDecoder::~HTTPDecoder() {
	// TODO Auto-generated destructor stub
}

Request HTTPDecoder::decode(char* request) {

	Request req;
	std::string raw = std::string(request);
	//printf("%s\n", request);
	if(raw.find("GET") != std::string::npos)
	{
		req.type = HTTP_REQUEST_TYPE_GET;

	}
	else if(raw.find("POST") != std::string::npos)
	{
		req.type = HTTP_REQUEST_TYPE_POST;

		std::size_t foundMSG = raw.find("\n\r\n");
		std::string msg = "";

		for(unsigned long i = foundMSG+3; i<raw.length(); i++)
		{
			if(raw.at(i) == '\n')
			{
				break;
			}
			else
			{
				msg += raw.at(i);
			}
		}

		req.postMessage = msg;
		//printf("%s\n", msg.c_str());
	}

	std::size_t foundURL = raw.find("/");
	std::string url = "";
	for(unsigned long i = foundURL+1; i<raw.length(); i++)
	{
		if(raw.at(i) == ' ' || raw.at(i) == '?')
		{
			break;
		}
		else
		{
			url += raw.at(i);
		}
	}

	//printf("%s\n", url.c_str());
	req.url = url;

	return req;
}

Response HTTPDecoder::generateResponse(Request req)
{
	std::string file = req.url;
    if(file == "favicon.ico")
    {
        file = favicon;
    }
    else
    {
        favicon = file;
    }

    //printf("%s\n", file.c_str());

	std::string filePath = resourcePath + "/" + file;
	if(file == "" || file == " " || file == "/" || file == "/ ")
	{
		filePath = resourcePath + "/index.html";
	}

    Response response;

    FILE* filep;
    bool found = false;

    for(int i = 0; i < 2; i++)
    {
        filep = fopen(filePath.c_str(), "rb");
        if(filep != NULL)
        {
            found = true;
        }
        else
        {
            usleep(1000);
        }
    }

    if(found)
    {


        fseek(filep, 0, SEEK_END);

        unsigned long fileLen=ftell(filep);
        fseek(filep, 0, SEEK_SET);
        char* file_data;
        file_data=new char[fileLen+1];
        while (!feof(filep)) {
            fread(file_data, fileLen, 1, filep);
        }

        fclose(filep);

        std::string header = "HTTP/1.1 200 OK\n";
        std::string footer = "\n\n";

        char buf[50];
        time_t now = time(0);
        struct tm tm = *gmtime(&now);
        strftime(buf, sizeof buf, "%a, %d %b %Y %H:%M:%S %Z", &tm);
        //printf("%s\n", buf);
        header += "Date: " + std::string(buf) + "\n";
        header += "Server: JECCbot server\n";
        //header += "Last-Modified: Sat, 01 Feb 2020 10:34:00 GMT";
        header += "Content-Length: " + std::to_string(fileLen)+ "\n";
        //header += "Content-Type: text/html";
        header += "Connection: Closed";
        header += footer;

	//printf("%s\n", buf);

        //printf("%s", header.c_str());

        response.len = header.length()+ fileLen + footer.length();
        response.content = new char[response.len];

        for(unsigned long i = 0; i < header.length(); i++)
        {
            response.content[i] = header.at(i);
        }

        for(unsigned long i = 0; i < fileLen; i++)
        {
            response.content[i+header.length()] = file_data[i];
        }

        for(unsigned long i = 0; i < footer.length(); i++)
        {
            response.content[i+header.length()+fileLen] = footer.at(i);
        }

        /*for(int i = 0; i < response.len; i++)
        {
            printf("%c", response.content[i]);
        }*/
        //printf("%s\n", response.content);
    }
    else
    {
        response.content = new char[30];
        response.len = sprintf(response.content, "HTTP/1.1 404 Not Found\n\n");
    }

    return response;
}


