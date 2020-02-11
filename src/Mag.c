#include "Mag.h"

float xmag, ymag, zmag, tempC = 0.0f;

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

#if 0
	SPI_SetCSLow(SPI2_CS_PORT, SPI2_CS1);
	txBuff[0] = MAG_CTRL_REG2;
	txBuff[1] = MAG_CTRL_REG2_V;
	SPI_Tx(&SPI_Bus_2, &txBuff[0], 2);
	SPI_SetCSHi(SPI2_CS_PORT, SPI2_CS1);

	SPI_SetCSLow(SPI2_CS_PORT, SPI2_CS1);
	txBuff[0] = MAG_CTRL_REG3;
	txBuff[1] = MAG_CTRL_REG3_V;
	SPI_Tx(&SPI_Bus_2, &txBuff[0], 2);
	SPI_SetCSHi(SPI2_CS_PORT, SPI2_CS1);

	SPI_SetCSLow(SPI2_CS_PORT, SPI2_CS1);
	txBuff[0] = MAG_CTRL_REG4;
	txBuff[1] = MAG_CTRL_REG4_V;
	SPI_Tx(&SPI_Bus_2, &txBuff[0], 2);
	SPI_SetCSHi(SPI2_CS_PORT, SPI2_CS1);

	SPI_SetCSLow(SPI2_CS_PORT, SPI2_CS1);
	txBuff[0] = MAG_CTRL_REG5;
	txBuff[1] = MAG_CTRL_REG5_V;
	SPI_Tx(&SPI_Bus_2, &txBuff[0], 2);
	SPI_SetCSHi(SPI2_CS_PORT, SPI2_CS1);
#endif

	HAL_Delay(1);
}



#if 0
		if(TotalReadTicks % SAMPLE_RATE == 0)
		{
			if(GPSrxBuff[0] != 0xff) memcpy(&recData, &GPSrxBuff, 512);
			readMAG();
			magData[0] = rxBuff[14+1] | (rxBuff[15+1] << 8);
			magData[1] = rxBuff[16+1] | (rxBuff[17+1] << 8);
			magData[2] = rxBuff[18+1] | (rxBuff[19+1] << 8);
			magData[3] = rxBuff[20+1] | (rxBuff[21+1] << 8);

			xmag 	= (float)magData[0] 	* (MAG_SCALE 	/ 32768.0f);
			ymag 	= (float)magData[1] 	* (MAG_SCALE 	/ 32768.0f);
			zmag 	= (float)magData[2] 	* (MAG_SCALE 	/ 32768.0f);
		}
#endif
