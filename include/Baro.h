#ifndef BARO_H_ /* include guard */
#define BARO_H_

#include "System.h"

#define BARO_START_RX	24
#define BARO_BYTES_RX	8

uint16_t 	NVM_PAR_T1;
uint16_t 	NVM_PAR_T2;
int8_t	 	NVM_PAR_T3;
int16_t 	NVM_PAR_P1;
int16_t 	NVM_PAR_P2;
int8_t	 	NVM_PAR_P3;
int8_t	 	NVM_PAR_P4;
uint16_t	NVM_PAR_P5;
uint16_t	NVM_PAR_P6;
int8_t	 	NVM_PAR_P7;
int8_t	 	NVM_PAR_P8;
int16_t	 	NVM_PAR_P9;
int8_t	 	NVM_PAR_P10;
int8_t	 	NVM_PAR_P11;
double		PAR_T1;
double		PAR_T2;
double		PAR_T3;
double		PAR_P1;
double		PAR_P2;
double		PAR_P3;
double		PAR_P4;
double		PAR_P5;
double		PAR_P6;
double		PAR_P7;
double		PAR_P8;
double		PAR_P9;
double		PAR_P10;
double		PAR_P11;

SPI_Bus SPI_Bus_2;
uint8_t BARO_txBuff[64];
uint8_t BARO_rxBuff[64];

float altitude;
float altBias;
double seaLevelPress;

extern void readBARO(uint8_t * arr);
extern void accumulateBARO(uint8_t * arr);
extern void procBARO();
extern void initBARO();

extern float baro_get_altitude();
extern double baro_get_pressure();
extern double baro_get_temperature();
extern double baro_comp_temp(uint32_t uncomp_temp);
extern double baro_comp_press(uint32_t uncomp_press, double calib_temp);


#endif /* BARO_H_ */
