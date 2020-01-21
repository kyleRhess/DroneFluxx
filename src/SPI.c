#include "SPI.h"


void SPI_Initialize(SPI_Bus * _SPI, SPI_TypeDef * SPI_BUS, uint32_t BaudRatePrescaler, uint32_t FirstBit, uint32_t CLKPolarity)
{
	// Prepare the SPI bus
	if(SPI_BUS == SPI1)
	{
		__HAL_RCC_SPI1_CLK_ENABLE();
	}
	else if(SPI_BUS == SPI2)
	{
		__HAL_RCC_SPI2_CLK_ENABLE();
	}
	else
	{
		__HAL_RCC_SPI5_CLK_ENABLE();
	}

	_SPI->SPIBus.Instance = SPI_BUS;
	_SPI->SPIBus.Init.BaudRatePrescaler = BaudRatePrescaler;
	if(SPI_BUS == SPI1)
		_SPI->SPIBus.Init.CLKPhase = SPI_PHASE_1EDGE;
	else
		_SPI->SPIBus.Init.CLKPhase = SPI_PHASE_2EDGE;
	_SPI->SPIBus.Init.CLKPolarity = CLKPolarity;
	_SPI->SPIBus.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLED;
	_SPI->SPIBus.Init.DataSize = SPI_DATASIZE_8BIT;
	_SPI->SPIBus.Init.Direction = SPI_DIRECTION_2LINES;
	_SPI->SPIBus.Init.FirstBit = FirstBit;
	_SPI->SPIBus.Init.Mode = SPI_MODE_MASTER;
	_SPI->SPIBus.Init.NSS = SPI_NSS_SOFT;
	_SPI->SPIBus.Init.TIMode = SPI_TIMODE_DISABLED;

	if( HAL_SPI_Init(&_SPI->SPIBus) == HAL_OK)
	{
//		GPIO_InitTypeDef SPI_Pins;
//		SPI_Pins.Mode = GPIO_MODE_AF_PP;
//		SPI_Pins.Pull = GPIO_PULLUP;
//		SPI_Pins.Speed = GPIO_SPEED_HIGH;
//
//		if(SPI_BUS == SPI1)
//		{
//			__GPIOB_CLK_ENABLE();
//			SPI_Pins.Pin = GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5;
//			SPI_Pins.Alternate = GPIO_AF5_SPI1;
//			HAL_GPIO_Init((GPIO_TypeDef *)GPIOB, &SPI_Pins);
//		}
//		else if(SPI_BUS == SPI2)
//		{
//			__GPIOC_CLK_ENABLE();
//			SPI_Pins.Pin = GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_7;
//			SPI_Pins.Alternate = GPIO_AF5_SPI2;
//			HAL_GPIO_Init((GPIO_TypeDef *)GPIOC, &SPI_Pins);
//		}
//		else
//		{
////			__GPIOB_CLK_ENABLE();
////			SPI_Pins.Pin = GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5;
////			SPI_Pins.Alternate = GPIO_AF5_SPI1;
////			HAL_GPIO_Init((GPIO_TypeDef *)GPIOB, &SPI_Pins);
//		}
	}
}

void SPI_Initialize_CS(GPIO_TypeDef * CSPort, uint32_t pin)
{
	// Prepare /CS lines
	if(     CSPort == GPIOA) __GPIOA_CLK_ENABLE();
	else if(CSPort == GPIOB) __GPIOB_CLK_ENABLE();
	else if(CSPort == GPIOC) __GPIOC_CLK_ENABLE();
	else if(CSPort == GPIOH) __GPIOH_CLK_ENABLE();

	GPIO_InitTypeDef CSpin;
	CSpin.Mode = GPIO_MODE_OUTPUT_PP;
	CSpin.Pin = pin;
	CSpin.Pull = GPIO_PULLUP;
	CSpin.Speed = GPIO_SPEED_HIGH;
	HAL_GPIO_Init(CSPort, &CSpin);

	SPI_SetCSHi(CSPort, pin);
}

void SPI_TxRx(SPI_Bus * _SPI, uint8_t *pTxData, uint8_t *pRxData, uint16_t bytesToTx)
{
	HAL_SPI_TransmitReceive(&_SPI->SPIBus, pTxData, pRxData, bytesToTx, HAL_MAX_DELAY);
}

void SPI_Tx(SPI_Bus * _SPI, uint8_t *pTxData, uint8_t bytesToTx)
{
	HAL_SPI_Transmit(&_SPI->SPIBus, pTxData, bytesToTx, HAL_MAX_DELAY);
}

void SPI_SetCSLow(GPIO_TypeDef *GPIOx, uint32_t Pin)
{
	GPIOx->BSRR = (uint32_t)(Pin << 16);
}

void SPI_SetCSHi(GPIO_TypeDef *GPIOx, uint32_t Pin)
{
	GPIOx->BSRR = (uint32_t)(Pin);
}

void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi)
{
	GPIO_InitTypeDef SPI_Pins;
	SPI_Pins.Mode = GPIO_MODE_AF_PP;
	SPI_Pins.Pull = GPIO_PULLUP;
	SPI_Pins.Speed = GPIO_SPEED_HIGH;

	if(hspi->Instance == SPI1)
	{
		__GPIOB_CLK_ENABLE();
		SPI_Pins.Pin = GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5;
		SPI_Pins.Alternate = GPIO_AF5_SPI1;
		HAL_GPIO_Init((GPIO_TypeDef *)GPIOB, &SPI_Pins);
	}
	else if(hspi->Instance == SPI2)
	{
		__GPIOC_CLK_ENABLE();
		SPI_Pins.Pin = GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_7;
		SPI_Pins.Alternate = GPIO_AF5_SPI2;
		HAL_GPIO_Init((GPIO_TypeDef *)GPIOC, &SPI_Pins);
	}
	else
	{
//			__GPIOB_CLK_ENABLE();
//			SPI_Pins.Pin = GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5;
//			SPI_Pins.Alternate = GPIO_AF5_SPI1;
//			HAL_GPIO_Init((GPIO_TypeDef *)GPIOB, &SPI_Pins);
	}
}
