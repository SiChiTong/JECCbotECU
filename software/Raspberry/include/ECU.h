#ifndef ECU_H
#define ECU_H

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <thread>
#include <math.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <fcntl.h>
#include <sys/signal.h>
#include <errno.h>
#include <termios.h>
#include <mutex>
#include <assert.h>
#include <string>
#include <signal.h>

#include "ECUProtocol.h"
#include "ECUBlobs.h"

#define ECU_UART_DELAY 1000
#define ECU_UART_TIMEOUT 30

class ECU
{
    public:
        ECU(char* port, int baud);
        ~ECU();

        int16_t read16(uint16_t address);
        int32_t read32(uint16_t address);
        float readFloat(uint16_t address);

        bool write16(uint16_t address, int16_t value);
        bool write32(uint16_t address, int32_t value);
        bool writeFloat(uint16_t address, float value);

        NmeaBlob getNmeaData();

    protected:

    private:

        char* port;
        int baud;

        int fd;
        struct termios term;

        std::string writeStr(char* str, int len, int timeout);
        std::string readValStr(uint16_t address, uint8_t instruction);

        std::thread *workerThread;
        bool workerThreadStop;
        void worker();

        std::mutex mu;

        NmeaBlob nmeaBlob;
        void fillNmeaBlob();
};

#endif // ECU_H
