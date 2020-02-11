#include "IMU.h"
#include "System.h"

/*
 * Register Address/Value pairs types definitions
 */
typedef uint8_t address;
typedef uint8_t value;
typedef struct {
	address addr;
	value val;
} imu_reg;

float zacc, xacc, yacc 							= 0.0f;
float zgyr, xgyr, ygyr 							= 0.0f;
float zgyrBias, xgyrBias, ygyrBias, altBias 	= 0.0f;
static int16_t imuDataCount 			= 0;

/*
 * Register Address/Value pairs
 */
const imu_reg initReg[] = {
	{(IMU_WHO_AM_I | 0x80), 	0}, // First reads chip ID
	{IMU_FIFO_3, 	IMU_FIFO_3_V},
	{IMU_FIFO_5, 	IMU_FIFO_5_V},
	{IMU_CTRL1_XL, 	IMU_CTRL1_XL_V},
	{IMU_CTRL2_G, 	IMU_CTRL2_G_V},
	{IMU_CTRL3_C, 	IMU_CTRL3_C_V},
	{IMU_CTRL4_C, 	IMU_CTRL4_C_V},
	{IMU_CTRL5_C, 	IMU_CTRL5_C_V},
	{IMU_CTRL6_C, 	IMU_CTRL6_C_V},
	{IMU_CTRL7_G, 	IMU_CTRL7_G_V},
	{IMU_CTRL8_XL, 	IMU_CTRL8_XL_V},
	{0x1A, 			0x00},
	{0x18, 			0x00},
	{0x19, 			0x00},
	{0x73, 			0x00},
	{0x74, 			0x00},
	{0x75, 			0x00},
	{0, 			0}
};

/*
 * Read IMU data
 */
void readIMU(uint8_t * arr)
{
	IMU_txBuff[0] = (IMU_OUT_TEMP_L | 0x80); // Set read bit
	IMU_txBuff[1] = 0;
	IMU_txBuff[2] = 0;
	IMU_txBuff[3] = 0;
	IMU_txBuff[4] = 0;
	IMU_txBuff[5] = 0;
	IMU_txBuff[6] = 0;
	IMU_txBuff[7] = 0;
	IMU_txBuff[8] = 0;
	IMU_txBuff[9] = 0;
	IMU_txBuff[10] = 0;
	IMU_txBuff[11] = 0;
	IMU_txBuff[12] = 0;
	IMU_txBuff[13] = 0;
	IMU_txBuff[14] = 0;

	SPI_SendReceive(&SPI_Bus_2, SPI2_CS_PORT, SPI2_CS0, &IMU_txBuff[0], &arr[IMU_START_RX], IMU_BYTES_TX);
}

void accumulateIMU(uint8_t * arr)
{
	// Got data in rxBuffer so store it.
	imuData[TEMPC] 		= arr[0+1]  | (arr[1+1] 	<< 8);
	imuData[GYRO_X] 	= arr[2+1]  | (arr[3+1] 	<< 8);
	imuData[GYRO_Y] 	= arr[4+1]  | (arr[5+1] 	<< 8);
	imuData[GYRO_Z] 	= arr[6+1]  | (arr[7+1] 	<< 8);
	imuData[ACCEL_X] 	= arr[8+1]  | (arr[9+1] 	<< 8);
	imuData[ACCEL_Y] 	= arr[10+1] | (arr[11+1] 	<< 8);
	imuData[ACCEL_Z] 	= arr[12+1] | (arr[13+1] 	<< 8);

	imuDataAccumulated[TEMPC] 		+= imuData[TEMPC];
	imuDataAccumulated[GYRO_X] 		+= imuData[GYRO_X];
	imuDataAccumulated[GYRO_Y] 		+= imuData[GYRO_Y];
	imuDataAccumulated[GYRO_Z] 		+= imuData[GYRO_Z];
	imuDataAccumulated[ACCEL_X] 	+= imuData[ACCEL_X];
	imuDataAccumulated[ACCEL_Y] 	+= imuData[ACCEL_Y];
	imuDataAccumulated[ACCEL_Z] 	+= imuData[ACCEL_Z];

	imuDataCount++;
}

void procIMU()
{
	if(imuDataCount > 0)
	{
		// Get average of SAMPLE_RATE/CONTROL_RATE samples.
		imuDataAccumulated[TEMPC] 		/= imuDataCount;
		imuDataAccumulated[GYRO_X] 		/= imuDataCount;
		imuDataAccumulated[GYRO_Y] 		/= imuDataCount;
		imuDataAccumulated[GYRO_Z] 		/= imuDataCount;
		imuDataAccumulated[ACCEL_X] 	/= imuDataCount;
		imuDataAccumulated[ACCEL_Y] 	/= imuDataCount;
		imuDataAccumulated[ACCEL_Z] 	/= imuDataCount;

		// Scale data to our ranges.
		xgyr 	= (float)imuDataAccumulated[GYRO_X] 	* (GYRO_SCALE 	/ 32768.0f);
		ygyr 	= (float)imuDataAccumulated[GYRO_Y] 	* (GYRO_SCALE 	/ 32768.0f);
		zgyr 	= (float)imuDataAccumulated[GYRO_Z] 	* (GYRO_SCALE 	/ 32768.0f);
		xacc 	= (float)imuDataAccumulated[ACCEL_X] 	* (ACCL_SCALE 	/ 32768.0f);
		yacc 	= (float)imuDataAccumulated[ACCEL_Y] 	* (ACCL_SCALE 	/ 32768.0f);
		zacc 	= (float)imuDataAccumulated[ACCEL_Z] 	* (ACCL_SCALE 	/ 32768.0f);
		tempC	= 25.0f + ((float)imuDataAccumulated[TEMPC] * (1.0f / 256.0f));

		for (uint8_t i = 0; i < sizeof(imuDataAccumulated)/sizeof(imuDataAccumulated[0]); ++i)
		{
			imuDataAccumulated[i] = 0;
		}
		imuDataCount = 0;

		// Fix biases.
		xgyr -= xgyrBias;
		ygyr -= ygyrBias;
		zgyr -= zgyrBias;

		// Apply some IIR filter to data.
	#define K_T 0.95f
		xgyro_Ahrs = (1.0f - K_T)*xgyro_Ahrs + (K_T * xgyr);
		ygyro_Ahrs = (1.0f - K_T)*ygyro_Ahrs + (K_T * ygyr);
		zgyro_Ahrs = (1.0f - K_T)*zgyro_Ahrs + (K_T * zgyr);
		xaccl_Ahrs = (1.0f - K_T)*xaccl_Ahrs + (K_T * xacc);
		yaccl_Ahrs = (1.0f - K_T)*yaccl_Ahrs + (K_T * yacc);
		zaccl_Ahrs = (1.0f - K_T)*zaccl_Ahrs + (K_T * zacc);
	}
}

/*
 * Initialize IMU registers and ensure we can read it
 */
void initIMU()
{
	int i = 0;
	while(initReg[i].addr != 0)
	{
		IMU_txBuff[0] = initReg[i].addr;
		IMU_txBuff[1] = initReg[i].val;

		if(i == 0)
		{
			SPI_SendReceive(&SPI_Bus_2, SPI2_CS_PORT, SPI2_CS0, &IMU_txBuff[0], &IMU_rxBuff[0], 2);
			while(IMU_rxBuff[1] != 0x6a){/*Catch failure to read*/}
		}
		else
		{
			SPI_Send(&SPI_Bus_2, SPI2_CS_PORT, SPI2_CS0, &IMU_txBuff[0], 2);
		}
		i++;
	}
	HAL_Delay(1);
}
