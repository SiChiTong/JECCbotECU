#ifndef DATALOGGER_H
#define DATALOGGER_H

#include <stdio.h>
#include <stdlib.h>
#include <string>

class DataLogger
{
    public:

        DataLogger(std::string resourcePath);
        ~DataLogger();

        void logString(std::string file, std::string data, bool clear);

    private:

        std::string resourcePath;
};

#endif
