#ifndef ECUPROTOCOL_H_INCLUDED
#define ECUPROTOCOL_H_INCLUDED

#define API_MEMORY_SIZE 400

#define API_STATE_JOYDRIVE 0x0000
#define API_STATE_MOVEHEADING 0x0001

#define API_INSTRUCTION_READ 0x02
#define API_INSTRUCTION_READ32 0x03
#define API_INSTRUCTION_WRITE 0x04

#define API_ERROR_WRONG_FORMAT 0x0000
#define API_ERROR_INVALID_ADDRESS 0x0001
#define API_ERROR_WRONG_INSTRUCTOR 0x0002
#define API_ERROR_ACCESS_DENIED 0x0003

#define API_REG_STATE 0x0000

#define API_REG_PWMLEFT 0x0002
#define API_REG_PWMRIGHT 0x0004

#define API_REG_MOVEHEADING_P 0x0005
#define API_REG_MOVEHEADING_SPEED 0x0006
#define API_REG_MOVEHEADING_HEADING 0x0007

#define API_BENCH_LIDAR_START 0x000f
#define API_BENCH_LIDAR_END API_BENCH_LIDAR_START + 360

#define API_REG_HEADING_KVH 0x0178

#define API_BENCH_GPS_START 0x0179
#define API_BENCH_GPS_END API_BENCH_GPS_START + 5

#endif // ECUPROTOCOL_H_INCLUDED
