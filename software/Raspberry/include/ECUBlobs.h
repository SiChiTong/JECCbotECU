#ifndef ECUBLOBS_H_INCLUDED
#define ECUBLOBS_H_INCLUDED

typedef struct NmeaBlob
{
    int16_t heading;
    int32_t time;
    float latitude;
    float longitude;
}NmeaBlob;

#endif // ECUBLOBS_H_INCLUDED
