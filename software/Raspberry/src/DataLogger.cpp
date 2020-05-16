#include "DataLogger.h"

DataLogger::DataLogger(std::string resourcePath)
{
    this->resourcePath = resourcePath;
}

DataLogger::~DataLogger(){}

void DataLogger::logString(std::string file, std::string data, bool clear)
{
    std::string path = resourcePath + "/" + file;

    data += "\n";

    FILE *fp;
    while(1)
    {
        if(clear)
        {
            fp = fopen(path.c_str(), "w");
        }
        else
        {
            fp = fopen(path.c_str(), "a");
        }

        if(fp != NULL)
        {
            fprintf(fp, "%s", data.c_str());
            fclose(fp);
            break;
        }
    }
}
