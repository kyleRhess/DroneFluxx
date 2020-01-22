#include "Reset.h"
#include "System.h"

static int rstCounter = 0;

/*
 * Setup the RST pin
 */
int Reset_Init()
{
	GPIO_InitTypeDef gRstPin;
	gRstPin.Pin = RST_PIN;
	gRstPin.Mode = GPIO_MODE_INPUT;
	gRstPin.Pull = GPIO_NOPULL;
	gRstPin.Speed = GPIO_SPEED_HIGH;
	HAL_GPIO_Init(GPIOB, &gRstPin);

	return HAL_OK;
}

/*
 * Return: True after RST button held for over MILLIS milliseconds
 */
bool Reset_Check()
{
	bool rc = false;

	// Get current pin state
	if((GPIOB->IDR & RST_PIN) != (uint32_t)GPIO_PIN_RESET)
	{
		rstCounter = 0;
	}
	else
	{
		rstCounter++;
		if(rstCounter > ((1000/CONTROL_RATE) * MILLIS)) rc = true;
	}

	return rc;
}
