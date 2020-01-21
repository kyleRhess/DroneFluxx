#include "Data.h"
#include "System.h"
#include "minmea.h"

static TIM_HandleTypeDef SamplingTimer = { .Instance = TIM9 };
SPI_Bus SPI_Bus_2;
SPI_Bus SPI_Bus_1;

static bool newDat = false;
static int counter = 0;

volatile uint16_t ReadTicks = 0;
volatile uint32_t TotalReadTicks = 0;
volatile uint32_t msgReadTicks = 0;

int initData()
{
	for (size_t i = 0; i < sizeof(scaledData)/sizeof(scaledData[0]); ++i)
	{
		scaledData[i] = 0.0f;
	}

	for (size_t i = 0; i < sizeof(accumMagData)/sizeof(accumMagData[0]); ++i)
	{
		accumMagData[i] = 0;
	}

	memset(txBuff, 0, sizeof(txBuff));
	memset(Data_rxBuff, 0, sizeof(Data_rxBuff));

	InitSPIBus();

	return HAL_OK;
}

/*
 * All this should do is read SPI data and store it.
 * Nothing else. Called from ISR at 5 kHz.
 */
uint8_t recData[512];
float maxZ, minZ;
char line[MINMEA_MAX_LENGTH];
int timeCount = 0;
char* msgId, msgGPSData, NoMsg, msgNo, NoSv;
void getRawData()
{
	if(SYSTEM_STATE != SYS_STATE_RESET)
	{
		readIMU();

#if 0
		GpsReadSize = initGPS();
		if(GpsReadSize > -1)
		{
			memset(line, 0, sizeof(line));
			strncpy(line, GPSrxBuff, GpsReadSize);


			switch (minmea_sentence_id(line, false))
			{
				case MINMEA_SENTENCE_RMC:
				{
					struct minmea_sentence_rmc frame;
					if (minmea_parse_rmc(&frame, line))
					{
						timeCount++;
						if(timeCount > 4)
						{
							printf("$RMC: raw coordinates and speed: (%d/%d,%d/%d) %d/%d\n",
									frame.latitude.value, frame.latitude.scale,
									frame.longitude.value, frame.longitude.scale,
									frame.speed.value, frame.speed.scale);
							printf("$RMC fixed-point coordinates and speed scaled to three decimal places: (%d,%d) %d\n",
									minmea_rescale(&frame.latitude, 1000),
									minmea_rescale(&frame.longitude, 1000),
									minmea_rescale(&frame.speed, 1000));
							printf("$RMC floating point degree coordinates and speed: (%f,%f) %f\n\n",
									minmea_tocoord(&frame.latitude),
									minmea_tocoord(&frame.longitude),
									minmea_tofloat(&frame.speed));
							timeCount = 0;
						}
					}
				} break;

				case MINMEA_SENTENCE_GGA:
				{
					struct minmea_sentence_gga frame;
					if (minmea_parse_gga(&frame, line))
					{
						numSVs = (float)frame.satellites_tracked;
					}
				} break;

				case MINMEA_SENTENCE_GLL:
				{
					struct minmea_sentence_gll frame;
					if (minmea_parse_gll(&frame, line))
					{
						if(frame.status == 'A')
						{
							aircraft_WriteLED(LED_B, HIGH);
							aircraft_WriteLED(LED_C, LOW);
							lat = minmea_tocoord(&frame.latitude);
							lon = minmea_tocoord(&frame.longitude);
						}
						else
						{
							aircraft_WriteLED(LED_B, LOW);
							aircraft_WriteLED(LED_C, HIGH);
						}
					}
				} break;

				case MINMEA_SENTENCE_VTG:
				{
					struct minmea_sentence_vtg frame;
					if (minmea_parse_vtg(&frame, line))
					{
						groundSpeed = minmea_tofloat(&frame.speed_knots) * 1.150783f;
						heading = minmea_tofloat(&frame.true_track_degrees);
					}
				} break;
			}






			if(strcmp(msgId, "$GPGSV") == 0)
			{
				msgId = strtok(NULL, ",");
				msgId = strtok(NULL, ",");
				msgId = strtok(NULL, ",");

				if(strcmp(msgId, "0") != 0)
				{
					while(msgId != NULL)
					{
						msgId = strtok(NULL, ",");
					}
				}
			}
		}
#endif

		if(TotalReadTicks % 5000 == 0)
		{
			//if(GPSrxBuff[0] != 0xff) memcpy(&recData, &GPSrxBuff, 512);
//			readMAG();
//			magData[0] = rxBuff[14+1] | (rxBuff[15+1] << 8);
//			magData[1] = rxBuff[16+1] | (rxBuff[17+1] << 8);
//			magData[2] = rxBuff[18+1] | (rxBuff[19+1] << 8);
//			magData[3] = rxBuff[20+1] | (rxBuff[21+1] << 8);
//
//			xmag 	= (float)magData[0] 	* (MAG_SCALE 	/ 32768.0f);
//			ymag 	= (float)magData[1] 	* (MAG_SCALE 	/ 32768.0f);
//			zmag 	= (float)magData[2] 	* (MAG_SCALE 	/ 32768.0f);
		}

		// Read baro at 5000/100 Hz
		if(TotalReadTicks % 100 == 0)
		{
			readBARO();

			uint32_t uncompTemp, uncompPress;
			uint32_t msb, lsb, xlsb;

			xlsb = (uint32_t)Data_rxBuff[BARO_START_RX+2];
			lsb = (uint32_t)Data_rxBuff[BARO_START_RX+3] << 8;
			msb = (uint32_t)Data_rxBuff[BARO_START_RX+4] << 16;
			uncompPress = msb | lsb | xlsb;

			xlsb = (uint32_t)Data_rxBuff[BARO_START_RX+5];
			lsb = (uint32_t)Data_rxBuff[BARO_START_RX+6] << 8;
			msb = (uint32_t)Data_rxBuff[BARO_START_RX+7] << 16;
			uncompTemp = msb | lsb | xlsb;

			baro_comp_temp(uncompTemp);
			baro_comp_press(uncompPress);

#define		A	45.0
			if(seaLevelPress == -99.0 && calib_press > 10.0)
			{
				// Pressure at sea-level
				seaLevelPress = (calib_press / (pow((-1.0*(((A-44330.0)/44330.0))), (1051.0/200.0)) * 100.0));
			}
			altitude = 44330.0f * (1.0f - (float)pow(((calib_press / 100.0) / seaLevelPress), 0.190295));
		}

		// Got data in rxBuffer so store it
		imuData[0] = Data_rxBuff[0+1]  | (Data_rxBuff[1+1] << 8);
		imuData[1] = Data_rxBuff[2+1]  | (Data_rxBuff[3+1] << 8);
		imuData[2] = Data_rxBuff[4+1]  | (Data_rxBuff[5+1] << 8);
		imuData[3] = Data_rxBuff[6+1]  | (Data_rxBuff[7+1] << 8);
		imuData[4] = Data_rxBuff[8+1]  | (Data_rxBuff[9+1] << 8);
		imuData[5] = Data_rxBuff[10+1] | (Data_rxBuff[11+1] << 8);
		imuData[6] = Data_rxBuff[12+1] | (Data_rxBuff[13+1] << 8);

		// Don't over-sample the data. Need BW!
		xgyr 	= imuData[GYRO_X] 	* (GYRO_SCALE 	/ 32768.0f);
		ygyr 	= imuData[GYRO_Y] 	* (GYRO_SCALE 	/ 32768.0f);
		zgyr 	= imuData[GYRO_Z] 	* (GYRO_SCALE 	/ 32768.0f);
		xacc 	= imuData[ACCEL_X] 	* (ACCL_SCALE 	/ 32768.0f);
		yacc 	= imuData[ACCEL_Y] 	* (ACCL_SCALE 	/ 32768.0f);
		zacc 	= imuData[ACCEL_Z] 	* (ACCL_SCALE 	/ 32768.0f);
		tempC	= 25.0f + (imuData[0]		* (1.0f / 256.0f));

		// Fix biases
		xgyr -= xgyrBias;
		ygyr -= ygyrBias;
		zgyr -= zgyrBias;

#define K_T 0.80f
		if(K_T < 0.98f)
		{
			xgyro_Ahrs = (1.0f - K_T)*xgyro_Ahrs + (K_T * xgyr);
			ygyro_Ahrs = (1.0f - K_T)*ygyro_Ahrs + (K_T * ygyr);
			zgyro_Ahrs = (1.0f - K_T)*zgyro_Ahrs + (K_T * zgyr);
		}
		else
		{
			xgyro_Ahrs = xgyr;
			ygyro_Ahrs = ygyr;
			zgyro_Ahrs = zgyr;
		}

		if(zgyro_Ahrs < minZ) minZ = zgyro_Ahrs;
		if(zgyro_Ahrs > maxZ) maxZ = zgyro_Ahrs;

		ReadTicks++;
		TotalReadTicks++;
		msgReadTicks++;

		// accumulate new data for averaging
		for (size_t i = 0; i < sizeof(magData)/sizeof(magData[0]); ++i)
		{
			accumMagData[i] += magData[i];
		}

		// Flag the controller to process the accumulated data
		if(ReadTicks == (SAMPLE_RATE/CONTROL_RATE))
		{
			SYSTEM_STATE = SYS_STATE_PROCESSING;
		}

		// Run the filter on the current inertial measures @ SAMPLE_RATE
		if(SYS_READ_TIME(TotalReadTicks) > 0.1f)
		{
	#ifdef IMU_ONLY
			MadgwickAHRSupdateIMU(xgyr * (PI / 180.0f), ygyr * (PI / 180.0f), zgyr * (PI / 180.0f), xacc, yacc, zacc);
	#else
			MadgwickAHRSupdate(   xgyr * (PI / 180.0f), ygyr * (PI / 180.0f), zgyr * (PI / 180.0f), xacc, yacc, zacc, -xmag, -ymag, zmag);
	#endif
		}


		//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
	}
}

/*
 * Run the data processor at F = SAMPLE_RATE/CONTROL_RATE  Hz
 */
void procRawData()
{
	// Average the accum'd data
//	for (size_t i = 0; i < sizeof(accumMagData)/sizeof(accumMagData[0]); ++i)
//	{
//		scaledData[i] = (float)((float)accumMagData[i] / (float)ReadTicks);
//	}
//
//	// Scale gyros, accels, mags, tempC
//	xmag 	= scaledData[MAG_X] 	* (MAG_SCALE 	/ 32768.0f);
//	ymag 	= scaledData[MAG_Y] 	* (MAG_SCALE 	/ 32768.0f);
//	zmag 	= scaledData[MAG_Z] 	* (MAG_SCALE 	/ 32768.0f);
//	tempC 	= 21.0f + (float)scaledData[TEMPC] / 8.0f;
//
//	// Zero-out accumulator
//	for (size_t i = 0; i < sizeof(accumData)/sizeof(accumData[0]); ++i)
//	{
//		accumData[i] = 0;
//	}
	ReadTicks = 0;

	// Compute roll/pitch/yaw angles from quaternions
	toEulerianAngle();
	roll_Ahrs 	= pitch_Madgwick;
	pitch_Ahrs 	= roll_Madgwick;
	yaw_Ahrs 	= yaw_Madgwick;

	delta_pitch_Ahrs = 	(pitch_Ahrs 	- last_pitch_Ahrs	) / 0.001f;
	delta_roll_Ahrs = 	(roll_Ahrs 		- last_roll_Ahrs	) / 0.001f;
	delta_yaw_Ahrs = 	(yaw_Ahrs 		- last_yaw_Ahrs		) / 0.001f;

	last_pitch_Ahrs = pitch_Ahrs;
	last_roll_Ahrs = roll_Ahrs;
	last_yaw_Ahrs = yaw_Ahrs;

	// Tell system to run control loop
	SYSTEM_STATE = SYS_STATE_CONTROLLING;
}

/*
 * Read IMU data
 */
void readIMU()
{
	SPI_SetCSLow(SPI2_CS_PORT, SPI2_CS0);
	txBuff[0] = (IMU_OUT_TEMP_L | 0x80); // Set read bit
	txBuff[1] = 0;
	txBuff[2] = 0;
	txBuff[3] = 0;
	txBuff[4] = 0;
	txBuff[5] = 0;
	txBuff[6] = 0;
	txBuff[7] = 0;
	txBuff[8] = 0;
	txBuff[9] = 0;
	txBuff[10] = 0;
	txBuff[11] = 0;
	txBuff[12] = 0;
	txBuff[13] = 0;
	txBuff[14] = 0;
	SPI_TxRx(&SPI_Bus_2, &txBuff[0], &Data_rxBuff[IMU_START_RX], IMU_BYTES_TX);
	SPI_SetCSHi(SPI2_CS_PORT, SPI2_CS0);
}

/*
 * Read mag data
 */
void readMAG()
{
	SPI_SetCSLow(SPI2_CS_PORT, SPI2_CS1);
	txBuff[0] = (MAG_OUT_X_L | 0x80 | 0x40); // Set read bit & auto-inc bit
	txBuff[1] = 0;
	txBuff[2] = 0;
	txBuff[3] = 0;
	txBuff[4] = 0;
	txBuff[5] = 0;
	txBuff[6] = 0;
	txBuff[7] = 0;
	txBuff[8] = 0;
	SPI_TxRx(&SPI_Bus_2, &txBuff[0], &Data_rxBuff[MAG_START_RX], MAG_BYTES_TX);
	SPI_SetCSHi(SPI2_CS_PORT, SPI2_CS1);
}

/*
 * Read baro data
 */
void readBARO()
{
	SPI_SetCSLow(SPI2_CS_PORT, SPI2_CS2);
	txBuff[0] = (BARO_PDATA_0 | 0x80); // Set read bit
	txBuff[1] = 0; // dummy byte
	txBuff[2] = 0; // press 7-0
	txBuff[3] = 0; // press 15-8
	txBuff[4] = 0; // press 23-16
	txBuff[5] = 0; // temp
	txBuff[6] = 0;
	txBuff[7] = 0;
	SPI_TxRx(&SPI_Bus_2, &txBuff[0], &Data_rxBuff[BARO_START_RX], BARO_BYTES_RX);
	SPI_SetCSHi(SPI2_CS_PORT, SPI2_CS2);
}

double baro_comp_temp(uint32_t uncomp_temp)
{
	double partial_1;
	double partial_2;

	partial_1 = (double)(uncomp_temp - PAR_T1);
	partial_2 = (double)(partial_1 * PAR_T2);

	calib_temp = partial_2 + (partial_1 * partial_1) * PAR_T3;

	return calib_temp;
}

double baro_comp_press(uint32_t uncomp_press)
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

	calib_press = partial_out1 + partial_out2 + partial_4;

	return calib_press;
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void InitSPIBus()
{
	//
	// Prepare the SPI buses
	//
	SPI_Initialize(&SPI_Bus_2, SPI2, SPI_BAUDRATEPRESCALER_16, SPI_FIRSTBIT_MSB, SPI_POLARITY_HIGH);
	SPI_Initialize_CS(SPI2_CS_PORT, SPI2_CS0);
	SPI_Initialize_CS(SPI2_CS_PORT, SPI2_CS1);
	SPI_Initialize_CS(SPI2_CS_PORT, SPI2_CS2);

	SPI_Initialize(&SPI_Bus_1, SPI1, SPI_BAUDRATEPRESCALER_32, SPI_FIRSTBIT_MSB, SPI_POLARITY_HIGH);
	SPI_Initialize_CS(SPI1_CS_PORT, SPI1_CS0);


	HAL_NVIC_SetPriority(SPI1_IRQn, 2, 0);
	HAL_NVIC_EnableIRQ(SPI1_IRQn);
	HAL_NVIC_SetPriority(SPI2_IRQn, 3, 0);
	HAL_NVIC_EnableIRQ(SPI2_IRQn);


	initIMU();
	HAL_Delay(5);
//	initMAG();
//	HAL_Delay(5);
	initBARO();
	HAL_Delay(5);
}


static int con = 0;
int initGPS()
{
//	memset(GPStxBuff, 0xff, sizeof(GPStxBuff));


	uint8_t ffbytes = 0;
	newDat = false;
	counter = 0;

	if(con == 0)
	{
		memset(GPSrxBuff, 0, sizeof(GPSrxBuff));
	}


	SPI_SetCSLow(SPI1_CS_PORT, SPI1_CS0);
	SPI_TxRx(&SPI_Bus_1, 0xff, &GPSrxBuff[con++], 1);
	SPI_SetCSHi(SPI1_CS_PORT, SPI1_CS0);

	if(GPSrxBuff[0] == 0xff)
	{
		con = 0;
	}
	else if(GPSrxBuff[con - 1] == 0xff || (GPSrxBuff[con - 1] == 0xa && GPSrxBuff[con - 2] == 0xd))
	{
		// We've read a whole message
		//HAL_UART_Transmit_IT(&s_UARTHandle, GPSrxBuff, con);
		int rc = con;
		con = 0;
		return rc;
	}

	return -1;
}

void initIMU()
{
	SPI_SetCSLow(SPI2_CS_PORT, SPI2_CS0);
	txBuff[0] = (IMU_WHO_AM_I | 0x80);
	txBuff[1] = 0;
	SPI_TxRx(&SPI_Bus_2, &txBuff[0], &Data_rxBuff[0], 2);
	SPI_SetCSHi(SPI2_CS_PORT, SPI2_CS0);

	HAL_Delay(5);

	while(Data_rxBuff[1] != 0x6a)
	{
		// Catch failure to read
	}

	// Send accel & gyro config. commands
	SPI_SetCSLow(SPI2_CS_PORT, SPI2_CS0);
	txBuff[0] = IMU_CTRL4_C;
	txBuff[1] = IMU_CTRL4_C_V;
	SPI_Tx(&SPI_Bus_2, &txBuff[0], 2);
	SPI_SetCSHi(SPI2_CS_PORT, SPI2_CS0);

	HAL_Delay(5);

	SPI_SetCSLow(SPI2_CS_PORT, SPI2_CS0);
	txBuff[0] = IMU_CTRL3_C;
	txBuff[1] = IMU_CTRL3_C_V;
	SPI_Tx(&SPI_Bus_2, &txBuff[0], 2);
	SPI_SetCSHi(SPI2_CS_PORT, SPI2_CS0);

	HAL_Delay(5);

	SPI_SetCSLow(SPI2_CS_PORT, SPI2_CS0);
	txBuff[0] = IMU_FIFO_3;
	txBuff[1] = IMU_FIFO_3_V;
	SPI_Tx(&SPI_Bus_2, &txBuff[0], 2);
	SPI_SetCSHi(SPI2_CS_PORT, SPI2_CS0);

	HAL_Delay(5);

	SPI_SetCSLow(SPI2_CS_PORT, SPI2_CS0);
	txBuff[0] = IMU_FIFO_5;
	txBuff[1] = IMU_FIFO_5_V;
	SPI_Tx(&SPI_Bus_2, &txBuff[0], 2);
	SPI_SetCSHi(SPI2_CS_PORT, SPI2_CS0);

	HAL_Delay(5);

	SPI_SetCSLow(SPI2_CS_PORT, SPI2_CS0);
	txBuff[0] = IMU_CTRL1_XL;
	txBuff[1] = IMU_CTRL1_XL_V;
	SPI_Tx(&SPI_Bus_2, &txBuff[0], 2);
	SPI_SetCSHi(SPI2_CS_PORT, SPI2_CS0);

	HAL_Delay(5);

	SPI_SetCSLow(SPI2_CS_PORT, SPI2_CS0);
	txBuff[0] = IMU_CTRL2_G;
	txBuff[1] = IMU_CTRL2_G_V;
	SPI_Tx(&SPI_Bus_2, &txBuff[0], 2);
	SPI_SetCSHi(SPI2_CS_PORT, SPI2_CS0);

	HAL_Delay(5);

	SPI_SetCSLow(SPI2_CS_PORT, SPI2_CS0);
	txBuff[0] = IMU_CTRL5_C;
	txBuff[1] = IMU_CTRL5_C_V;
	SPI_Tx(&SPI_Bus_2, &txBuff[0], 2);
	SPI_SetCSHi(SPI2_CS_PORT, SPI2_CS0);

	HAL_Delay(5);

	SPI_SetCSLow(SPI2_CS_PORT, SPI2_CS0);
	txBuff[0] = IMU_CTRL6_C;
	txBuff[1] = IMU_CTRL6_C_V;
	SPI_Tx(&SPI_Bus_2, &txBuff[0], 2);
	SPI_SetCSHi(SPI2_CS_PORT, SPI2_CS0);

	HAL_Delay(5);

	SPI_SetCSLow(SPI2_CS_PORT, SPI2_CS0);
	txBuff[0] = IMU_CTRL7_G;
	txBuff[1] = IMU_CTRL7_G_V;
	SPI_Tx(&SPI_Bus_2, &txBuff[0], 2);
	SPI_SetCSHi(SPI2_CS_PORT, SPI2_CS0);

	SPI_SetCSLow(SPI2_CS_PORT, SPI2_CS0);
	txBuff[0] = IMU_CTRL8_XL;
	txBuff[1] = IMU_CTRL8_XL_V;
	SPI_Tx(&SPI_Bus_2, &txBuff[0], 2);
	SPI_SetCSHi(SPI2_CS_PORT, SPI2_CS0);

	SPI_SetCSLow(SPI2_CS_PORT, SPI2_CS0);
	txBuff[0] = 0x1A;
	txBuff[1] = 0x00;
	SPI_Tx(&SPI_Bus_2, &txBuff[0], 2);
	SPI_SetCSHi(SPI2_CS_PORT, SPI2_CS0);

	SPI_SetCSLow(SPI2_CS_PORT, SPI2_CS0);
	txBuff[0] = 0x18;
	txBuff[1] = 0x00;
	SPI_Tx(&SPI_Bus_2, &txBuff[0], 2);
	SPI_SetCSHi(SPI2_CS_PORT, SPI2_CS0);

	SPI_SetCSLow(SPI2_CS_PORT, SPI2_CS0);
	txBuff[0] = 0x19;
	txBuff[1] = 0x00;
	SPI_Tx(&SPI_Bus_2, &txBuff[0], 2);
	SPI_SetCSHi(SPI2_CS_PORT, SPI2_CS0);

	SPI_SetCSLow(SPI2_CS_PORT, SPI2_CS0);
	txBuff[0] = 0x73;
	txBuff[1] = 0x00;
	SPI_Tx(&SPI_Bus_2, &txBuff[0], 2);
	SPI_SetCSHi(SPI2_CS_PORT, SPI2_CS0);

	SPI_SetCSLow(SPI2_CS_PORT, SPI2_CS0);
	txBuff[0] = 0x74;
	txBuff[1] = 0x00;
	SPI_Tx(&SPI_Bus_2, &txBuff[0], 2);
	SPI_SetCSHi(SPI2_CS_PORT, SPI2_CS0);

	SPI_SetCSLow(SPI2_CS_PORT, SPI2_CS0);
	txBuff[0] = 0x75;
	txBuff[1] = 0x00;
	SPI_Tx(&SPI_Bus_2, &txBuff[0], 2);
	SPI_SetCSHi(SPI2_CS_PORT, SPI2_CS0);

	HAL_Delay(5);
}

void initMAG()
{
	SPI_SetCSLow(SPI2_CS_PORT, SPI2_CS1);
	txBuff[0] = (MAG_WHO_AM_I | 0x80);
	txBuff[1] = 0;
	SPI_TxRx(&SPI_Bus_2, &txBuff[0], &Data_rxBuff[0], 2);
	SPI_SetCSHi(SPI2_CS_PORT, SPI2_CS1);
	while(Data_rxBuff[1] != 0x3d)
	{
		// Catch failure to read
	}

	// Send mag config. commands
	SPI_SetCSLow(SPI2_CS_PORT, SPI2_CS1);
	txBuff[0] = MAG_CTRL_REG1;
	txBuff[1] = MAG_CTRL_REG1_V;
	SPI_Tx(&SPI_Bus_2, &txBuff[0], 2);
	SPI_SetCSHi(SPI2_CS_PORT, SPI2_CS1);

	SPI_SetCSLow(SPI2_CS_PORT, SPI2_CS1);
	txBuff[0] = (MAG_CTRL_REG1 | 0x80);
	txBuff[1] = 0;
	SPI_TxRx(&SPI_Bus_2, &txBuff[0], &Data_rxBuff[0], 2);
	SPI_SetCSHi(SPI2_CS_PORT, SPI2_CS1);

	SPI_SetCSLow(SPI2_CS_PORT, SPI2_CS1);
	txBuff[0] = (MAG_CTRL_REG1 | 0x80);
	txBuff[1] = 0;
	SPI_TxRx(&SPI_Bus_2, &txBuff[0], &Data_rxBuff[0], 2);
	SPI_SetCSHi(SPI2_CS_PORT, SPI2_CS1);
//
//	SPI_SetCSLow(SPI2_CS_PORT, SPI2_CS1);
//	txBuff[0] = MAG_CTRL_REG2;
//	txBuff[1] = MAG_CTRL_REG2_V;
//	SPI_Tx(&SPI_Bus_2, &txBuff[0], 2);
//	SPI_SetCSHi(SPI2_CS_PORT, SPI2_CS1);
//
//	SPI_SetCSLow(SPI2_CS_PORT, SPI2_CS1);
//	txBuff[0] = MAG_CTRL_REG3;
//	txBuff[1] = MAG_CTRL_REG3_V;
//	SPI_Tx(&SPI_Bus_2, &txBuff[0], 2);
//	SPI_SetCSHi(SPI2_CS_PORT, SPI2_CS1);
//
//	SPI_SetCSLow(SPI2_CS_PORT, SPI2_CS1);
//	txBuff[0] = MAG_CTRL_REG4;
//	txBuff[1] = MAG_CTRL_REG4_V;
//	SPI_Tx(&SPI_Bus_2, &txBuff[0], 2);
//	SPI_SetCSHi(SPI2_CS_PORT, SPI2_CS1);
//
//	SPI_SetCSLow(SPI2_CS_PORT, SPI2_CS1);
//	txBuff[0] = MAG_CTRL_REG5;
//	txBuff[1] = MAG_CTRL_REG5_V;
//	SPI_Tx(&SPI_Bus_2, &txBuff[0], 2);
//	SPI_SetCSHi(SPI2_CS_PORT, SPI2_CS1);

	HAL_Delay(1);
}

void initBARO()
{

	SPI_SetCSLow(SPI2_CS_PORT, SPI2_CS2);
	txBuff[0] = (BARO_IF_CONF | 0x80);
	txBuff[1] = 0;
	txBuff[2] = 0;
	SPI_TxRx(&SPI_Bus_2, &txBuff[0], &Data_rxBuff[0], 3);
	SPI_SetCSHi(SPI2_CS_PORT, SPI2_CS2);
	SPI_SetCSLow(SPI2_CS_PORT, SPI2_CS2);
	txBuff[0] = BARO_IF_CONF;
	txBuff[1] = ((Data_rxBuff[2] & ~0x07) | 0x00);
	SPI_Tx(&SPI_Bus_2, &txBuff[0], 2);
	SPI_SetCSHi(SPI2_CS_PORT, SPI2_CS2);

	while(Data_rxBuff[2] != 0x50)
	{
		SPI_SetCSLow(SPI2_CS_PORT, SPI2_CS2);
		txBuff[0] = (0x00 | 0x80);
		txBuff[1] = 0;
		txBuff[2] = 0;
		SPI_TxRx(&SPI_Bus_2, &txBuff[0], &Data_rxBuff[0], 3);
		SPI_SetCSHi(SPI2_CS_PORT, SPI2_CS2);

		HAL_Delay(5);
		// Catch failure to read
	}

	// Send baro config. commands
	SPI_SetCSLow(SPI2_CS_PORT, SPI2_CS2);
	txBuff[0] = (BARO_PWR_CTRL | 0x80);
	txBuff[1] = 0;
	txBuff[2] = 0;
	SPI_TxRx(&SPI_Bus_2, &txBuff[0], &Data_rxBuff[0], 3);
	SPI_SetCSHi(SPI2_CS_PORT, SPI2_CS2);
	SPI_SetCSLow(SPI2_CS_PORT, SPI2_CS2);
	txBuff[0] = BARO_PWR_CTRL;
	txBuff[1] = ((Data_rxBuff[2] & ~0x33) | 0x33); // press en. temp en. normal mode
	SPI_Tx(&SPI_Bus_2, &txBuff[0], 2);
	SPI_SetCSHi(SPI2_CS_PORT, SPI2_CS2);

	SPI_SetCSLow(SPI2_CS_PORT, SPI2_CS2);
	txBuff[0] = (BARO_OSR | 0x80);
	txBuff[1] = 0;
	txBuff[2] = 0;
	SPI_TxRx(&SPI_Bus_2, &txBuff[0], &Data_rxBuff[0], 3);
	SPI_SetCSHi(SPI2_CS_PORT, SPI2_CS2);
	SPI_SetCSLow(SPI2_CS_PORT, SPI2_CS2);
	txBuff[0] = BARO_OSR;
	txBuff[1] = ((Data_rxBuff[2] & ~0x3f) | 0x03); // press oversampling x8
	SPI_Tx(&SPI_Bus_2, &txBuff[0], 2);
	SPI_SetCSHi(SPI2_CS_PORT, SPI2_CS2);

	SPI_SetCSLow(SPI2_CS_PORT, SPI2_CS2);
	txBuff[0] = (BARO_ODR | 0x80);
	txBuff[1] = 0;
	txBuff[2] = 0;
	SPI_TxRx(&SPI_Bus_2, &txBuff[0], &Data_rxBuff[0], 3);
	SPI_SetCSHi(SPI2_CS_PORT, SPI2_CS2);
	SPI_SetCSLow(SPI2_CS_PORT, SPI2_CS2);
	txBuff[0] = BARO_ODR;
	txBuff[1] = ((Data_rxBuff[2] & ~0x1f) | 0x02); // press ODR 50 Hz
	SPI_Tx(&SPI_Bus_2, &txBuff[0], 2);
	SPI_SetCSHi(SPI2_CS_PORT, SPI2_CS2);

	SPI_SetCSLow(SPI2_CS_PORT, SPI2_CS2);
	txBuff[0] = (BARO_CONFIG | 0x80);
	txBuff[1] = 0;
	txBuff[2] = 0;
	SPI_TxRx(&SPI_Bus_2, &txBuff[0], &Data_rxBuff[0], 3);
	SPI_SetCSHi(SPI2_CS_PORT, SPI2_CS2);
	SPI_SetCSLow(SPI2_CS_PORT, SPI2_CS2);
	txBuff[0] = BARO_CONFIG;
	txBuff[1] = ((Data_rxBuff[2] & ~0x0e) | 0x02); // filter IIR is 3
	SPI_Tx(&SPI_Bus_2, &txBuff[0], 2);
	SPI_SetCSHi(SPI2_CS_PORT, SPI2_CS2);

	// Get the correction terms
	memset(txBuff, 0, sizeof(txBuff));
	memset(Data_rxBuff, 0, sizeof(Data_rxBuff));
	SPI_SetCSLow(SPI2_CS_PORT, SPI2_CS2);
	txBuff[0] = (0x31 | 0x80);
	SPI_TxRx(&SPI_Bus_2, &txBuff[0], &Data_rxBuff[0], 1+1+21);
	SPI_SetCSHi(SPI2_CS_PORT, SPI2_CS2);
	NVM_PAR_T1 = Data_rxBuff[2] | (Data_rxBuff[3] << 8);
	NVM_PAR_T2 = Data_rxBuff[4] | (Data_rxBuff[5] << 8);
	NVM_PAR_T3 = Data_rxBuff[6];
	NVM_PAR_P1 = Data_rxBuff[7] | (Data_rxBuff[8] << 8);
	NVM_PAR_P2 = Data_rxBuff[9] | (Data_rxBuff[10] << 8);
	NVM_PAR_P3 = Data_rxBuff[11];
	NVM_PAR_P4 = Data_rxBuff[12];
	NVM_PAR_P5 = Data_rxBuff[13] | (Data_rxBuff[14] << 8);
	NVM_PAR_P6 = Data_rxBuff[15] | (Data_rxBuff[16] << 8);
	NVM_PAR_P7 = Data_rxBuff[17];
	NVM_PAR_P8 = Data_rxBuff[18];
	NVM_PAR_P9 = Data_rxBuff[19] | (Data_rxBuff[20] << 8);
	NVM_PAR_P10 = Data_rxBuff[21];
	NVM_PAR_P11 = Data_rxBuff[22];
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
	memset(txBuff, 0, sizeof(txBuff));
	memset(Data_rxBuff, 0, sizeof(Data_rxBuff));

	HAL_Delay(1);
}

int InitSamplingTimer()
{
	__HAL_RCC_TIM9_CLK_ENABLE();
    SamplingTimer.Init.Prescaler = 250;
    SamplingTimer.Init.CounterMode = TIM_COUNTERMODE_UP;
    SamplingTimer.Init.Period = 80; // 5 kHz = 100E6/((250)*(80)*(1))
    SamplingTimer.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    if(HAL_TIM_Base_Init(&SamplingTimer) != HAL_OK) return HAL_ERROR;
    if(HAL_TIM_Base_Start_IT(&SamplingTimer) != HAL_OK) return HAL_ERROR;

    return HAL_OK;
}


void TIM1_BRK_TIM9_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&SamplingTimer);
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &SamplingTimer)
		getRawData();
}

