#include "System.h"
#include "Serial.h"

static bool errorSet = false;
UART_HandleTypeDef s_UARTHandle = { .Instance = USART1 };
__IO ITStatus UartReady				= SET;
__IO ITStatus UartRxCmdReady 		= RESET;

int InitSerial(uint32_t baudrate, uint32_t stopbits, uint32_t datasize, uint32_t parity)
{
	int rc = HAL_OK;

	s_UARTHandle.Init.BaudRate   = baudrate;
	s_UARTHandle.Init.WordLength = datasize;
	s_UARTHandle.Init.StopBits   = stopbits;
	s_UARTHandle.Init.Parity     = parity;
	s_UARTHandle.Init.Mode       = USART_MODE_TX_RX;
	s_UARTHandle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	s_UARTHandle.Init.OverSampling = UART_OVERSAMPLING_8;
	rc = HAL_UART_Init(&s_UARTHandle);

	HAL_UART_Receive_IT(&s_UARTHandle, uartRxBuffer, 1);

	datMsg.msgCnt = 0;
	serialODR 	= 50;
	rxIndex 	= 0;
	connLoss 	= 0;
	firstSync 	= 1;
	handshakeCMD = 0;
	serialMSG 	= 0;

	return rc;
}

void RunSerial()
{
	if(UartRxCmdReady == SET)
		handshakeCMD = procCmd();

	if(msgReadTicks >= MSG_RATE_HZ(serialODR))
	{
		// Only attempt tx if we are told to
		if(serialMSG != 0)
			transmitSerialData();

		// Limit ODR range
		if(serialODR > 1250) serialODR = 1250;
		if(serialODR < 1) serialODR = 1;

		if(errorSet)
		{
			rxIndex = 0;
			errorSet = false;
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET);
			memset(uartRx, 0, sizeof(uartRx));
		}
		msgReadTicks = 0;
	}
	receiveSerial();
}

void receiveSerial()
{
    HAL_UART_Receive_IT(&s_UARTHandle, uartRxBuffer, 1);
}

uint8_t getStatus(uint8_t msgCount)
{
	uint8_t rc = 0;
	uint32_t gainVal = 0;
	uint8_t floatIndex = 0;
	uint8_t shift = 0;

	if(msgCount == 0)
	{
		rc = handshakeCMD;
	}
	else if(msgCount < 49) // PID Gains
	{
		if(msgCount < 17) 		msgCount -= 0;	//1	->16
		else if(msgCount < 33) 	msgCount -= 16; //17->32
		else if(msgCount < 49) 	msgCount -= 32; //33->48

		floatIndex = (msgCount - 1)/4;
		shift = (8*(msgCount - (floatIndex*4)) - 8);

		if(msgCount < 17) 		memcpy(&gainVal, &proportionalGain[floatIndex], 4);
		else if(msgCount < 33) 	memcpy(&gainVal, &integralGain[floatIndex], 4);
		else if(msgCount < 49) 	memcpy(&gainVal, &derivativeGain[floatIndex], 4);

		rc = (uint8_t)((gainVal >> shift) & 0x000000ff);
	}
	return rc;
}

void transmitSerialData()
{
	switch (serialMSG)
	{
		case SERIAL_MSG_DRONE:
		{
			float arrt[7] =
			{
				pitch_Ahrs, // Test data
				0.0f,
				0.0f,
				0.0f,
				0.0f,
				0.0f,
				0.0f
			};

			static uint32_t chksum32 = 0;
			chksum32 = 0;
			datMsg.start = SERIAL_MSG_START;
			chksum32 += datMsg.start;
			for (int i = 0; i < 7; ++i)
			{
				memcpy(&datMsg.dat[i], &arrt[i], 4);
				chksum32 += (datMsg.dat[i] & 0xff000000) >> 24;
				chksum32 += (datMsg.dat[i] & 0x00ff0000) >> 16;
				chksum32 += (datMsg.dat[i] & 0x0000ff00) >> 8;
				chksum32 += (datMsg.dat[i] & 0x000000ff) >> 0;
			}
			if(datMsg.msgCnt >= 255)
				datMsg.msgCnt = 0;
			else
				datMsg.msgCnt++;

			chksum32 += datMsg.msgCnt;

			datMsg.statusA = datMsg.msgCnt;
			datMsg.statusB = (getStatus(datMsg.msgCnt));

			chksum32 += datMsg.statusA;
			chksum32 += datMsg.statusB;

			datMsg.cksum = (0xff - (uint8_t)(chksum32 & 0x000000ff));

			uartBuffer = (uint8_t*)&datMsg;
			HAL_UART_Transmit_IT(&s_UARTHandle, uartBuffer, sizeof(struct DataMsg));
			break;
		}
		case SERIAL_MSG_GPS:
			if(GpsReadSize > 1)
			{
				HAL_UART_Transmit_IT(&s_UARTHandle, GPSrxBuff, GpsReadSize);
				GpsReadSize = 0;
			}
			break;
		default:
			break;
	}
}

static uint32_t cksum = 0;
uint8_t procCmd()
{
	uint8_t rc = 0;
	if(uartRx[0] == SERIAL_CMD_START)
	{
		uint8_t payloadLen = uartRx[2];
		if(uartRx[payloadLen + 4] == SERIAL_CMD_END)
		{
			// Good start & end flags
			cksum = 0;
			for(uint16_t i = 1; i < (3+payloadLen); i++)
			{
				cksum += uartRx[i];
			}
			cksum = (0xff - (cksum & 0x000000ff));
			if(cksum == uartRx[payloadLen + 3])
			{
				// Good CRC
				uint8_t cmd = uartRx[1];
				uint8_t payload[payloadLen];

				for (int i = 0; i < payloadLen; ++i)
				{
					payload[i] = uartRx[i+3];
				}

				rc = cmd;
				switch (cmd)
				{
					case CMD_SET_OUT_DAT:
						serialMSG = payload[0];
						break;
					case CMD_SET_OUT_RATE:
						serialODR = payload[0] << 8 | payload[1];
						break;
					case CMD_SET_PID_GAIN:
						decodePIDGains(payload);
						break;
					default:
						break;
				}
			}
		}
	}

	// Blank out rx buff after processing
	//memset(&uartRx, 0, RX_BUFF_SZ);
	rxIndex = 0;
	UartRxCmdReady = RESET;
	return rc;
}

void decodePIDGains(uint8_t *payload)
{
    int startI = 0;
	for (int i = 0; i < 4; ++i)
	{
		proportionalGain[i] = toFloat(payload, startI);
		startI += 4;
	}

	for (int i = 0; i < 4; ++i)
	{
		integralGain[i] = toFloat(payload, startI);
		startI += 4;
	}

	for (int i = 0; i < 4; ++i)
	{
		derivativeGain[i] = toFloat(payload, startI);
		startI += 4;
	}
}

float toFloat(uint8_t bytes[], int startI)
{
	uint32_t rcUint = (uint32_t)((bytes[3+startI] << 24) |
								 (bytes[2+startI] << 16) |
								 (bytes[1+startI] << 8) |
								 (bytes[0+startI] << 0));
	float rcFloat = *((float*)&rcUint);
	return rcFloat;
}

uint8_t calcCRC(uint8_t datArr[], size_t size)
{
	uint32_t crc = 0;
	for(size_t i = 0; i < size-1; i++)
		crc += datArr[i];
	crc = (0xff - (crc & 0x000000ff));
	return (uint8_t)crc;
}


/*
 * UART Interrupts
 */
static int len = 0;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
	if(UartHandle->Instance == USART1)
	{
		if(UartRxCmdReady == RESET)
		{
			// copy ISR buffer into RX buffer
			uartRx[rxIndex++] = uartRxBuffer[0];

			// find number of incoming bytes
			if(rxIndex == 3) len = uartRx[2];
			if(rxIndex == (len + 5)) UartRxCmdReady = SET;

			if(rxIndex >= 255)
			{
				memset(&uartRx, 0, RX_BUFF_SZ);
				rxIndex = 0;
				UartRxCmdReady = RESET;
			}

			HAL_UART_Receive_IT(&s_UARTHandle, uartRxBuffer, 1);
		}
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle)
{
	if(UartHandle->Instance == USART1)
	{
		UartReady = RESET;
	}
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *UartHandle)
{
	if(UartHandle->Instance == USART1)
	{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);
		errorSet = true;
		if(__HAL_UART_GET_FLAG(UartHandle, UART_FLAG_ORE) != RESET)
		{
			if(__HAL_UART_GET_FLAG(UartHandle, UART_FLAG_RXNE) == RESET)
			{
				UartHandle->Instance->DR;
			}
		}
	}
}

void USART1_IRQHandler(void)
{
//	 HAL_DMA_IRQHandler(&hdma_usart1_rx);
	  HAL_UART_IRQHandler(&s_UARTHandle);
}

void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{
	if(huart->Instance == USART1)
	{
		GPIO_InitTypeDef GPIO_InitStructureUart = {0};

		__HAL_RCC_USART1_CLK_ENABLE();
		__HAL_RCC_GPIOB_CLK_ENABLE();

		GPIO_InitStructureUart.Pin = GPIO_PIN_6 | GPIO_PIN_7;
		GPIO_InitStructureUart.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStructureUart.Alternate = GPIO_AF7_USART1;
		GPIO_InitStructureUart.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStructureUart.Pull = GPIO_PULLUP;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStructureUart);

		HAL_NVIC_SetPriority(USART1_IRQn, 4, 0);
		HAL_NVIC_EnableIRQ(USART1_IRQn);
	}
}
