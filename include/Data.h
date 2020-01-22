#ifndef DATA_H_ /* include guard */
#define DATA_H_

#include "System.h"

//#define USING_MAG

// Linear Acceleration: mg per LSB
#define LSM9DS0_ACCEL_MG_LSB_2G 		(0.061F)
#define LSM9DS0_ACCEL_MG_LSB_4G 		(0.122F)
#define LSM9DS0_ACCEL_MG_LSB_6G 		(0.183F)
#define LSM9DS0_ACCEL_MG_LSB_8G 		(0.244F)
#define LSM9DS0_ACCEL_MG_LSB_16G 		(0.732F) // Is this right? Was expecting 0.488F

// Magnetic Field Strength: gauss range
#ifdef USING_MAG
#define LSM9DS0_MAG_MGAUSS_2GAUSS      	(0.08F)
#define LSM9DS0_MAG_MGAUSS_4GAUSS      	(0.16F)
#define LSM9DS0_MAG_MGAUSS_8GAUSS      	(0.32F)
#define LSM9DS0_MAG_MGAUSS_12GAUSS     	(0.48F)
#endif

// Angular Rate: dps per LSB
#define LSM9DS0_GYRO_DPS_DIGIT_245DPS   (0.00875F)
#define LSM9DS0_GYRO_DPS_DIGIT_500DPS   (0.01750F)
#define LSM9DS0_GYRO_DPS_DIGIT_2000DPS  (0.07000F)

// Temperature: LSB per degree celsius
#define LSM9DS0_TEMP_LSB_DEGREE_CELSIUS (8)  // 1°C = 8, 25° = 200, etc.

#define IMU_START_RX	0
#define IMU_BYTES_TX	15
#define MAG_START_RX	15
#define MAG_BYTES_TX	9
#define BARO_START_RX	24
#define BARO_BYTES_RX	8

uint8_t GPSrxBuff[1024];
int GpsReadSize;
uint8_t txBuff[64];
uint8_t Data_rxBuff[64];
volatile int16_t imuData[10];
#ifdef USING_MAG
volatile int16_t magData[4];
volatile long accumMagData[4];
#endif
extern volatile uint16_t ReadTicks;
extern volatile uint32_t TotalReadTicks;
extern volatile uint32_t msgReadTicks;
float scaledData[10];

// Baro compensation
double calib_temp;
double calib_press;

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

void getRawData(void);
void procRawData(void);
int InitSamplingTimer(void);
void InitSPIBus(void);
void initIMU();
void initMAG();
void initBARO();
int initGPS();
int initData(void);
void readIMU(void);
void readMAG(void);
void readBARO(void);
double baro_comp_temp(uint32_t uncomp_temp);
double baro_comp_press(uint32_t uncomp_press);

#endif /* DATA_H_ */
