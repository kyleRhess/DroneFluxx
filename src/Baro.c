#include "Baro.h"
#include "System.h"
#include "SPI.h"

#define		INIT_ALT	45.0

static double calib_temp 	= 0.0;
static double calib_press 	= 0.0;
static uint32_t accumTemp, accumPress = 0;
float altitude 				= 0.0f;
double seaLevelPress 		= -99.0;
static int16_t baroDataCount = 0;

float baro_get_altitude()
{
	return altitude;
}

double baro_get_pressure()
{
	return calib_press;
}

double baro_get_temperature()
{
	return calib_temp;
}

/*
 * Read baro data
 */
void readBARO(uint8_t * arr)
{
	BARO_txBuff[0] = (BARO_PDATA_0 | 0x80); // Set read bit
	BARO_txBuff[1] = 0; // dummy byte
	BARO_txBuff[2] = 0; // press 7-0
	BARO_txBuff[3] = 0; // press 15-8
	BARO_txBuff[4] = 0; // press 23-16
	BARO_txBuff[5] = 0; // temp
	BARO_txBuff[6] = 0;
	BARO_txBuff[7] = 0;

	SPI_SendReceive(&SPI_Bus_2, SPI2_CS_PORT, SPI2_CS2, &BARO_txBuff[0], &arr[BARO_START_RX], BARO_BYTES_RX);
}

/*
 * Accumulate the data we read.
 */
void accumulateBARO(uint8_t * arr)
{
	static uint32_t msb, lsb, xlsb;
	xlsb 		= (uint32_t)arr[BARO_START_RX+2];
	lsb 		= (uint32_t)arr[BARO_START_RX+3] << 8;
	msb 		= (uint32_t)arr[BARO_START_RX+4] << 16;
	accumPress  += msb | lsb | xlsb;
	xlsb 		= (uint32_t)arr[BARO_START_RX+5];
	lsb 		= (uint32_t)arr[BARO_START_RX+6] << 8;
	msb 		= (uint32_t)arr[BARO_START_RX+7] << 16;
	accumTemp 	+= msb | lsb | xlsb;
	baroDataCount++;
}

/*
 * Process the data we read into pressure & temperature.
 */
void procBARO()
{
	if(baroDataCount > 0)
	{
		static uint32_t uncompTemp, uncompPress;
		uncompPress = accumPress / baroDataCount;
		uncompTemp 	= accumTemp / baroDataCount;

		calib_temp 	= baro_comp_temp(uncompTemp);
		calib_press = baro_comp_press(uncompPress, calib_temp);

		// Pressure at sea-level
		if(seaLevelPress == -99.0 && baro_get_pressure() > 10.0)
			seaLevelPress = (baro_get_pressure() / (pow((-1.0*(((INIT_ALT-44330.0)/44330.0))), (1051.0/200.0)) * 100.0));
		altitude = 44330.0f * (1.0f - (float)pow(((baro_get_pressure() / 100.0) / seaLevelPress), 0.190295));

		baroDataCount = 0;
		accumTemp = 0;
		accumPress = 0;
	}
}

/*
 * Setup the baro registers and read compensation terms from baro.
 */
void initBARO()
{
	BARO_txBuff[0] = (BARO_IF_CONF | 0x80);
	BARO_txBuff[1] = 0;
	BARO_txBuff[2] = 0;
	SPI_SendReceive(&SPI_Bus_2, SPI2_CS_PORT, SPI2_CS2, &BARO_txBuff[0], &BARO_rxBuff[0], 3);
	BARO_txBuff[0] = BARO_IF_CONF;
	BARO_txBuff[1] = ((BARO_rxBuff[2] & ~0x07) | 0x00);
	SPI_SendReceive(&SPI_Bus_2, SPI2_CS_PORT, SPI2_CS2, &BARO_txBuff[0], &BARO_rxBuff[0], 2);

	BARO_txBuff[0] = (0x00 | 0x80);
	BARO_txBuff[1] = 0;
	BARO_txBuff[2] = 0;
	SPI_SendReceive(&SPI_Bus_2, SPI2_CS_PORT, SPI2_CS2, &BARO_txBuff[0], &BARO_rxBuff[0], 3);
	while(BARO_rxBuff[2] != 0x50){/*Catch failure to read*/}

	BARO_txBuff[0] = (BARO_PWR_CTRL | 0x80);
	BARO_txBuff[1] = 0;
	BARO_txBuff[2] = 0;
	SPI_SendReceive(&SPI_Bus_2, SPI2_CS_PORT, SPI2_CS2, &BARO_txBuff[0], &BARO_rxBuff[0], 3);
	BARO_txBuff[0] = BARO_PWR_CTRL;
	BARO_txBuff[1] = ((BARO_rxBuff[2] & ~0x33) | 0x33); // press en. temp en. normal mode
	SPI_SendReceive(&SPI_Bus_2, SPI2_CS_PORT, SPI2_CS2, &BARO_txBuff[0], &BARO_rxBuff[0], 2);

	BARO_txBuff[0] = (BARO_OSR | 0x80);
	BARO_txBuff[1] = 0;
	BARO_txBuff[2] = 0;
	SPI_SendReceive(&SPI_Bus_2, SPI2_CS_PORT, SPI2_CS2, &BARO_txBuff[0], &BARO_rxBuff[0], 3);
	BARO_txBuff[0] = BARO_OSR;
	BARO_txBuff[1] = ((BARO_rxBuff[2] & ~0x3f) | 0x03); // press oversampling x8
	SPI_SendReceive(&SPI_Bus_2, SPI2_CS_PORT, SPI2_CS2, &BARO_txBuff[0], &BARO_rxBuff[0], 2);

	BARO_txBuff[0] = (BARO_ODR | 0x80);
	BARO_txBuff[1] = 0;
	BARO_txBuff[2] = 0;
	SPI_SendReceive(&SPI_Bus_2, SPI2_CS_PORT, SPI2_CS2, &BARO_txBuff[0], &BARO_rxBuff[0], 3);
	BARO_txBuff[0] = BARO_ODR;
	BARO_txBuff[1] = ((BARO_rxBuff[2] & ~0x1f) | 0x02); // press ODR 50 Hz
	SPI_SendReceive(&SPI_Bus_2, SPI2_CS_PORT, SPI2_CS2, &BARO_txBuff[0], &BARO_rxBuff[0], 2);

	BARO_txBuff[0] = (BARO_CONFIG | 0x80);
	BARO_txBuff[1] = 0;
	BARO_txBuff[2] = 0;
	SPI_SendReceive(&SPI_Bus_2, SPI2_CS_PORT, SPI2_CS2, &BARO_txBuff[0], &BARO_rxBuff[0], 3);
	BARO_txBuff[0] = BARO_CONFIG;
	BARO_txBuff[1] = ((BARO_rxBuff[2] & ~0x0e) | 0x02); // filter IIR is 3
	SPI_SendReceive(&SPI_Bus_2, SPI2_CS_PORT, SPI2_CS2, &BARO_txBuff[0], &BARO_rxBuff[0], 2);

	HAL_Delay(1);

	// Get the correction terms
	memset(BARO_txBuff, 0, sizeof(BARO_txBuff));
	memset(BARO_rxBuff, 0, sizeof(BARO_rxBuff));
	BARO_txBuff[0] = (0x31 | 0x80);
	SPI_SendReceive(&SPI_Bus_2, SPI2_CS_PORT, SPI2_CS2, &BARO_txBuff[0], &BARO_rxBuff[0], 1+1+21);
	NVM_PAR_T1 = BARO_rxBuff[2] | (BARO_rxBuff[3] << 8);
	NVM_PAR_T2 = BARO_rxBuff[4] | (BARO_rxBuff[5] << 8);
	NVM_PAR_T3 = BARO_rxBuff[6];
	NVM_PAR_P1 = BARO_rxBuff[7] | (BARO_rxBuff[8] << 8);
	NVM_PAR_P2 = BARO_rxBuff[9] | (BARO_rxBuff[10] << 8);
	NVM_PAR_P3 = BARO_rxBuff[11];
	NVM_PAR_P4 = BARO_rxBuff[12];
	NVM_PAR_P5 = BARO_rxBuff[13] | (BARO_rxBuff[14] << 8);
	NVM_PAR_P6 = BARO_rxBuff[15] | (BARO_rxBuff[16] << 8);
	NVM_PAR_P7 = BARO_rxBuff[17];
	NVM_PAR_P8 = BARO_rxBuff[18];
	NVM_PAR_P9 = BARO_rxBuff[19] | (BARO_rxBuff[20] << 8);
	NVM_PAR_P10 = BARO_rxBuff[21];
	NVM_PAR_P11 = BARO_rxBuff[22];
	PAR_T1 = (double)NVM_PAR_T1 / (pow(2.0, -8.0));
	PAR_T2 = (double)NVM_PAR_T2 / (pow(2.0, 30.0));
	PAR_T3 = (double)NVM_PAR_T3 / (pow(2.0, 48.0));
	PAR_P1 = ((double)NVM_PAR_P1 - pow(2.0, 14.0)) / (pow(2.0, 20.0));
	PAR_P2 = ((double)NVM_PAR_P2 - pow(2.0, 14.0)) / (pow(2.0, 29.0));
	PAR_P3 = (double)NVM_PAR_P3 / (pow(2.0, 32.0));
	PAR_P4 = (double)NVM_PAR_P4 / (pow(2.0, 37.0));
	PAR_P5 = (double)NVM_PAR_P5 / (pow(2.0, -3.0));
	PAR_P6 = (double)NVM_PAR_P6 / (pow(2.0, 6.0));
	PAR_P7 = (double)NVM_PAR_P7 / (pow(2.0, 8.0));
	PAR_P8 = (double)NVM_PAR_P8 / (pow(2.0, 15.0));
	PAR_P9 = (double)NVM_PAR_P9 / (pow(2.0, 48.0));
	PAR_P10 = (double)NVM_PAR_P10 / (pow(2.0, 48.0));
	PAR_P11 = (double)NVM_PAR_P11 / (pow(2.0, 65.0));
	memset(BARO_txBuff, 0, sizeof(BARO_txBuff));
	memset(BARO_rxBuff, 0, sizeof(BARO_rxBuff));

	HAL_Delay(1);
}

/*
 * Compensate baro temperature.
 */
double baro_comp_temp(uint32_t uncomp_temp)
{
	double partial_1;
	double partial_2;

	partial_1 = (double)(uncomp_temp - PAR_T1);
	partial_2 = (double)(partial_1 * PAR_T2);

	return partial_2 + (partial_1 * partial_1) * PAR_T3;
}

/*
 * Compensate baro pressure.
 */
double baro_comp_press(uint32_t uncomp_press, double calib_temp)
{
	double partial_1;
	double partial_2;
	double partial_3;
	double partial_4;
	double partial_out1;
	double partial_out2;

	partial_1 = PAR_P6 * calib_temp;
	partial_2 = PAR_P7 * (calib_temp * calib_temp);
	partial_3 = PAR_P8 * (calib_temp * calib_temp * calib_temp);
	partial_out1 = PAR_P5 + partial_1 + partial_2 + partial_3;

	partial_1 = PAR_P2 * calib_temp;
	partial_2 = PAR_P3 * (calib_temp * calib_temp);
	partial_3 = PAR_P4 * (calib_temp * calib_temp * calib_temp);
	partial_out2 = ((double)uncomp_press) * (PAR_P1 + partial_1 + partial_2 + partial_3);

	partial_1 = (double)uncomp_press * (double)uncomp_press;
	partial_2 = PAR_P9 + (PAR_P10 * calib_temp);
	partial_3 = partial_1 * partial_2;
	partial_4 = partial_3 + (((double)uncomp_press * (double)uncomp_press * (double)uncomp_press) * PAR_P11);

	return partial_out1 + partial_out2 + partial_4;
}
