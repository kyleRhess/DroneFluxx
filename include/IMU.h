#ifndef IMU_H_ /* include guard */
#define IMU_H_

#include "System.h"

// IMU Register addresses and values
#define IMU_WHO_AM_I	0x0f
#define IMU_FIFO_3		0x08
#define IMU_FIFO_3_V	0x00
#define IMU_FIFO_5		0x0a
#define IMU_FIFO_5_V	0x00
#define IMU_CTRL1_XL	0x10
#define IMU_CTRL1_XL_V	0xa6
#define IMU_CTRL2_G		0x11
#define IMU_CTRL2_G_V	0xac
#define IMU_CTRL3_C		0x12
#define IMU_CTRL3_C_V	0x44
#define IMU_CTRL4_C		0x13
#define IMU_CTRL4_C_V	0x06
#define IMU_CTRL5_C		0x14
#define IMU_CTRL5_C_V	0x00
#define IMU_CTRL6_C		0x15
#define IMU_CTRL6_C_V	0x02 //LPF = 173 Hz
#define IMU_CTRL7_G		0x16
#define IMU_CTRL7_G_V	0x00
#define IMU_CTRL8_XL	0x17
#define IMU_CTRL8_XL_V	0x00

// IMU Data register addresses
#define IMU_OUT_TEMP_L	0x20
#define IMU_OUT_TEMP_H	0x21
#define IMU_OUTX_L_G	0x22
#define IMU_OUTX_H_G	0x23
#define IMU_OUTY_L_G	0x24
#define IMU_OUTY_H_G	0x25
#define IMU_OUTZ_L_G	0x26
#define IMU_OUTZ_H_G	0x27
#define IMU_OUTX_L_XL	0x28
#define IMU_OUTX_H_XL	0x29
#define IMU_OUTY_L_XL	0x2A
#define IMU_OUTY_H_XL	0x2B
#define IMU_OUTZ_L_XL	0x2C
#define IMU_OUTZ_H_XL	0x2D

// Sensor indexes
#define MAG_X  			3
#define MAG_Y  			4
#define MAG_Z  			5

#define TEMPC    		0
#define GYRO_X  		1
#define GYRO_Y  		2
#define GYRO_Z  		3
#define ACCEL_X 		4
#define ACCEL_Y 		5
#define ACCEL_Z 		6
#define SEN_NUM 		10

// SPI definitions
#define IMU_START_RX	0
#define IMU_BYTES_TX	15

// Global sensor values
extern float zacc;
extern float yacc;
extern float xacc;
extern float zgyr;
extern float ygyr;
extern float xgyr;
extern float zgyrBias;
extern float ygyrBias;
extern float xgyrBias;
extern float tempC;

SPI_Bus SPI_Bus_2;
uint8_t IMU_txBuff[64];
uint8_t IMU_rxBuff[64];

extern void readIMU(uint8_t * arr);
extern void accumulateIMU(uint8_t * arr);
extern void procIMU();
extern void initIMU();


#endif /* IMU_H_ */
