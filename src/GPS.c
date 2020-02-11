#include "GPS.h"

static bool newDat = false;
static int counter = 0;

float lat, lon, heading, groundSpeed, numSVs 	= 0.0f;

/*
 * Initialize GPS module
 */
int initGPS()
{
	static int con = 0;
//	memset(GPStxBuff, 0xff, sizeof(GPStxBuff));

	newDat = false;
	counter = 0;

	if(con == 0)
	{
		memset(GPSrxBuff, 0, sizeof(GPSrxBuff));
	}

	SPI_SetCSLow(SPI1_CS_PORT, SPI1_CS0);
	SPI_TxRx(&SPI_Bus_1, (uint8_t *)0xff, &GPSrxBuff[con++], 1);
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
