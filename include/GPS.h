#ifndef GPS_H_ /* include guard */
#define GPS_H_

#include "System.h"
#include "minmea.h"

SPI_Bus SPI_Bus_1;

uint8_t recData[512];
char line[MINMEA_MAX_LENGTH];
int timeCount = 0;
char* msgId, msgGPSData, NoMsg, msgNo, NoSv;

extern float lat, lon, heading, groundSpeed, numSVs;

int initGPS();

#endif /* GPS_H_ */
