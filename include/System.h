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
#include "IMU.h"
#include "Baro.h"

#define IMU_ENABLED
//#define IMU_MAGS
#define GPS_ENABLED
#define BARO_ENABLED

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

// SPI port mapping
#define SPI1_CS_PORT	GPIOA
#define SPI1_CS0		GPIO_PIN_15
#define SPI2_CS_PORT	GPIOC
#define SPI2_CS0		GPIO_PIN_0
#define SPI2_CS1		GPIO_PIN_1
#define SPI2_CS2		GPIO_PIN_4

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

#define ERROR_IMU		0x01
#define ERROR_BARO		0x02
#define ERROR_GPS		0x04
#define ERROR_MAG		0x08


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

int InitializeSystem(void);
int InitPWMOutput(void);
int InitPID(void);

extern float mapVal(float x, float in_min, float in_max, float out_min, float out_max);


#endif /* SYSTEM_H_ */
