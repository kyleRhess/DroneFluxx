#ifndef SPI_H_ /* include guard */
#define SPI_H_

#include <stdio.h>
#include "stm32f4xx.h"

typedef struct SPI_Bus
{
	SPI_HandleTypeDef SPIBus;
} SPI_Bus;

void SPI_Initialize(SPI_Bus * _SPI, SPI_TypeDef * SPI_BUS, uint32_t BaudRatePrescaler, uint32_t FirstBit, uint32_t CLKPolarity);
void SPI_Initialize_CS(GPIO_TypeDef * CSPort, uint32_t pin);
void SPI_TxRx(SPI_Bus * _SPI, uint8_t *pTxData, uint8_t *pRxData, uint16_t bytesToTx);
void SPI_Tx(SPI_Bus * _SPI, uint8_t *pTxData, uint8_t bytesToTx);
void SPI_SetCSHi(GPIO_TypeDef * CSPort, uint32_t Pin);
void SPI_SetCSLow(GPIO_TypeDef * CSPort, uint32_t Pin);

#endif /* SPI_H_ */
