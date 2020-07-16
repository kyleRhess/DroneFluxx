#ifndef SERIAL_H_ /* include guard */
#define SERIAL_H_

#include <stdio.h>
#include "stm32f4xx.h"

#define MSG_RATE_HZ(xxx) 	(SAMPLE_RATE / xxx)

#define RX_BUFF_SZ 			4096

#define CMD_SET_OUT_DAT 	0x01
#define CMD_SET_OUT_RATE 	0x02
#define CMD_SET_PID_GAIN 	0x03

#define SERIAL_MSG_DRONE 	0x01
#define SERIAL_MSG_GPS	 	0x02

#define SERIAL_MSG_START 	0x4A
#define SERIAL_CMD_START 	0x7F
#define SERIAL_CMD_END 		0xF7

uint8_t *uartBuffer;
uint8_t uartRx[RX_BUFF_SZ];
uint8_t uartTx[RX_BUFF_SZ];
uint8_t uartRxBuffer[1];

uint16_t 	rxIndex;
uint16_t 	connLoss;
uint8_t 	firstSync;
uint16_t 	serialODR;
uint8_t 	serialMSG;
uint8_t		handshakeCMD;

struct DataMsg {
	uint8_t start;
	uint8_t msgCnt;
	uint32_t dat[7];
	uint8_t statusA;
	uint8_t statusB;
	uint8_t cksum;
} __attribute__((__packed__));

struct DataMsg datMsg;

int InitSerial(uint32_t baudrate, uint32_t stopbits, uint32_t datasize, uint32_t parity);
void RunSerial(void);
void transmitSerialData(void);
uint8_t getStatus(uint8_t msgCount);
uint8_t procCmd(void);
uint8_t calcCRC(uint8_t datArr[], size_t size);
float toFloat(uint8_t bytes[], int startI);
void decodePIDGains(uint8_t *payload);


#endif /* SERIAL_H_ */
