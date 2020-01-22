#ifndef SYSTEM_H_ /* include guard */
#define SYSTEM_H_

#include <stdio.h>
#include "stm32f4xx.h"
#include "SPI.h"
#include "PID.h"
#include "PWM.h"
#include "MadgwickAHRS.h"
#include "Data.h"
#include "Aircraft.h"
#include "Receiver.h"
#include "Reset.h"
#include "Serial.h"

#define IMU_ONLY
//#define IMU_MAGS

//#define PID_RATE

#define ATT_SCALE 1.0f
#define RATE_SCALE 3.0f

#define HIGH 			1
#define LOW  			0
typedef int bool;

#define SYS_READ_TIME(xxx) 	((float)(xxx) / (float)SAMPLE_RATE)
#define SQR(xxx) 			((xxx) * (xxx))

#define SAMPLE_RATE 	5000 //Hz
#define CONTROL_RATE 	1000 //Hz
#define PI 				3.1415926535897f
#define ACCL_SCALE		16.0f  	//g's
#define GYRO_SCALE 		2000.0f //dps
#define MAG_SCALE 		8.0f 	//gauss

#define MIN_THROT		6.0f
#define MIN_ESC_US		5.0f
#define MAX_ESC_US		25.0f
//#define IMU_ONLY

// SPI port mapping
#define SPI1_CS_PORT	GPIOA
#define SPI1_CS0		GPIO_PIN_15
#define SPI2_CS_PORT	GPIOC
#define SPI2_CS0		GPIO_PIN_0
#define SPI2_CS1		GPIO_PIN_1
#define SPI2_CS2		GPIO_PIN_4

// IMU Defs
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

// MAG Defs
#define MAG_WHO_AM_I	0x0f
#define MAG_CTRL_REG1	0x20
#define MAG_CTRL_REG1_V	0x7c
#define MAG_CTRL_REG2	0x21
#define MAG_CTRL_REG2_V	0x20
#define MAG_CTRL_REG3	0x22
#define MAG_CTRL_REG3_V	0x00
#define MAG_CTRL_REG4	0x23
#define MAG_CTRL_REG4_V	0x0c
#define MAG_CTRL_REG5	0x24
#define MAG_CTRL_REG5_V	0x00
#define MAG_OUT_X_L		0x28
#define MAG_OUT_X_H		0x29
#define MAG_OUT_Y_L		0x2a
#define MAG_OUT_Y_H		0x2b
#define MAG_OUT_Z_L		0x2c
#define MAG_OUT_Z_H		0x2d
#define MAG_TEMP_OUT_L	0x2e
#define MAG_TEMP_OUT_H	0x2f

// BARO Defs
#define BARO_WHO_AM_I	0x00
#define BARO_FIFO_CNFIG	0x17
#define BARO_PWR_CTRL	0x1B
#define BARO_OSR		0x1C
#define BARO_ODR		0x1D
#define BARO_CONFIG		0x1F
#define BARO_IF_CONF	0x1A
#define BARO_PDATA_0	0x04
#define BARO_PDATA_1	0x05
#define BARO_PDATA_2	0x06
#define BARO_TDATA_0	0x07
#define BARO_TDATA_1	0x08
#define BARO_TDATA_2	0x09
#define BARO_SENSTIME_0	0x0C
#define BARO_SENSTIME_1	0x0D
#define BARO_SENSTIME_2	0x0E

#define RWBIT 			0x7f
#define MSBIT 			0xbf

#define ACCEL_X 		4
#define ACCEL_Y 		5
#define ACCEL_Z 		6
#define MAG_X  			3
#define MAG_Y  			4
#define MAG_Z  			5
#define GYRO_X  		1
#define GYRO_Y  		2
#define GYRO_Z  		3
#define TEMPC    		9
#define SEN_NUM 		10

typedef enum
{
	MOTOR_FRONT_LEFT = 0,
	MOTOR_FRONT_RIGHT,
	MOTOR_BACK_LEFT,
	MOTOR_BACK_RIGHT,
	MOTOR_NUM
} MOTORS;

typedef enum
{
	PID_PITCH = 0,
	PID_ROLL,
	PID_YAW,
	PID_ALT,
	PID_NUM
} PID_CONTROLLERS_ANGLE;

typedef enum
{
	PID_XGYR = PID_PITCH,
	PID_YGYR,
	PID_ZGYR
} PID_CONTROLLERS_RATE;

typedef enum
{
	SYS_STATE_NONE = -1,
	SYS_STATE_IDLE,
	SYS_STATE_PROCESSING,
	SYS_STATE_CONTROLLING,
	SYS_STATE_RESET
} SYS_STATE;

extern volatile SYS_STATE SYSTEM_STATE;
extern float zacc;
extern float yacc;
extern float xacc;
extern float zgyr;
extern float ygyr;
extern float xgyr;
extern float zgyrBias;
extern float ygyrBias;
extern float xgyrBias;
extern float zmag;
extern float ymag;
extern float xmag;
extern float tempC;
extern float altitude;
extern float altBias;
extern double seaLevelPress;
extern float lat, lon, heading, groundSpeed, numSVs;
extern float channelPulseWidth_us[CHANNEL_NUM];
extern PWM_Out PWMtimer;

extern PID_Controller flightControl[PID_NUM];
extern float motorPower[MOTOR_NUM];

extern float proportionalGain[PID_NUM];
extern float integralGain[PID_NUM];
extern float derivativeGain[PID_NUM];

extern float proportionalGain[PID_NUM];
extern float integralGain[PID_NUM];
extern float derivativeGain[PID_NUM];

extern DMA_HandleTypeDef hdma_usart1_rx;

void RunSystem();
void runController();
void resetController();

void InitializeSystem(void);
int InitPWMOutput(void);
int InitPID(void);

extern float mapVal(float x, float in_min, float in_max, float out_min, float out_max);


#endif /* SYSTEM_H_ */
