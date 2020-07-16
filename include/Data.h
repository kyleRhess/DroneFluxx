#ifndef DATA_H_ /* include guard */
#define DATA_H_

#include "System.h"

//#define USING_MAG

#define MAG_START_RX	15
#define MAG_BYTES_TX	9

uint8_t GPSrxBuff[1024];
int GpsReadSize;
uint8_t txBuff[64];
uint8_t Data_rxBuff[64];
volatile int16_t imuData[10];
volatile int32_t imuDataAccumulated[10];
#ifdef USING_MAG
volatile int16_t magData[4];
volatile long accumMagData[4];
#endif
extern volatile uint16_t ReadTicks;
extern volatile uint32_t TotalReadTicks;
extern volatile uint32_t msgReadTicks;

void getRawData(void);
void procRawData(void);
int InitSamplingTimer(void);
int InitSPIBus(void);

int initData(void);

#endif /* DATA_H_ */
