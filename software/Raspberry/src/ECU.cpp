#include "ECU.h"

ECU::ECU(char* port, int baud)
{
    this->port = new char[strlen(port) + 1];
    strcpy(this->port, port);
    this->baud = baud;

    fd = open(port, O_RDWR | O_NOCTTY);
    if (fd == -1)
    {
        perror("Can't establish connection to JECCbot ECU...\n");
        //exit(1);
    }

    tcgetattr(fd,&term);
    cfsetispeed(&term,baud);
    term.c_cflag=baud|CS8|CLOCAL|CREAD;
    term.c_lflag =0;
    term.c_iflag = IGNPAR | IGNBRK | IGNCR;
    term.c_oflag =0;
    term.c_cc[VTIME]=1;
    term.c_cc[VMIN]=0;
    tcflush[fd,TCIFLUSH];
    tcsetattr(fd,TCSANOW,&term);

    workerThreadStop = false;
	workerThread = new std::thread(&ECU::worker, this);
//	workerThread->detach();
}

ECU::~ECU()
{
	workerThreadStop = true;
//	workerThread->join();

    close(fd);
}


std::string ECU::writeStr(char* str, int len, int timeout)
{
    write(fd, str, len);

    char receiveChar;
    std::string response = "";

    int counter = 0;

    while(receiveChar != '\n')
    {
        if(counter > timeout)
        {
            response = "";
            break;
        }
        read(fd, &receiveChar, 1);
        response += receiveChar;

        counter ++;
    }

    usleep(ECU_UART_DELAY);

    return response;
}

std::string ECU::readValStr(uint16_t address, uint8_t instruction)
{
    std::string value = "";

    if(address < API_MEMORY_SIZE)
    {
        char cmd[20];
        int len = sprintf(cmd, ":%02x%04x\n", instruction, address);

//        printf("%s", cmd);
        std::string response = writeStr(cmd, len, ECU_UART_TIMEOUT);
//        printf("%s", response.c_str());

        if(response.length() > 8 && response.at(0) == ':' && response.at(response.length() - 1) == '\n')
        {
            std::string addrStr = response.substr(1, 4);

            if(std::stoi(addrStr, nullptr, 16) == address)
            {
//                        printf("%s\n", addrStr.c_str());

                value = response.substr(5, response.length() - 2);
            }
        }
    }


    return value;
}

int16_t ECU::read16(uint16_t address)
{
    int16_t value = -1;

    std::string response = readValStr(address, API_INSTRUCTION_READ);

    if(response != "")
    {
        value = std::stoi(response, nullptr, 16);
    }

    return value;
}

int32_t ECU::read32(uint16_t address)
{
    int32_t value = -1;

    std::string response = readValStr(address, API_INSTRUCTION_READ32);

    if(response != "")
    {
        value = std::stoi(response, nullptr, 16);
    }

    return value;
}

float ECU::readFloat(uint16_t address)
{
    int32_t data = read32(address);

    float value;

    memcpy(&value, &data, 4);

    return value;
}

bool ECU::write16(uint16_t address, int16_t value)
{
    bool success = false;

    if(address < API_MEMORY_SIZE)
    {

        char cmd[20];
        char cmdRef[20];

        int len = sprintf(cmd, ":%02x%04x%04x\n", API_INSTRUCTION_WRITE, address, value);
        int lenRef = sprintf(cmdRef, ":%04x%04x\n", address, value);

        std::string response = writeStr(cmd, len, ECU_UART_TIMEOUT);

        if(strncmp(response.c_str(), cmdRef, lenRef) == 0)
        {
            success = true;
        }
    }

    return success;
}

bool ECU::write32(uint16_t address, int32_t value)
{

}

bool ECU::writeFloat(uint16_t address, float value)
{

}

void ECU::worker()
{
	while(!workerThreadStop)
	{
        fillNmeaBlob();
    }
}

NmeaBlob ECU::getNmeaData()
{
    return nmeaBlob;
}

void ECU::fillNmeaBlob()
{
    nmeaBlob.heading = read16(API_REG_HEADING_KVH);
    nmeaBlob.time = read32(API_BENCH_GPS_START);
    nmeaBlob.latitude = readFloat(API_BENCH_GPS_START + 2);
    nmeaBlob.longitude = readFloat(API_BENCH_GPS_START + 4);
}
