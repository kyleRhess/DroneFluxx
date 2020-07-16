#include "Data.h"
#include "System.h"
#include "IMU.h"
#include "Baro.h"

static TIM_HandleTypeDef SamplingTimer = { .Instance = TIM9 };
SPI_Bus SPI_Bus_2;
SPI_Bus SPI_Bus_1;

volatile uint16_t 	ReadTicks = 0;
volatile uint32_t 	TotalReadTicks = 0;
volatile uint32_t 	msgReadTicks = 0;
uint8_t 	Data_rxBuffA[64];

int initData()
{
	memset(Data_rxBuffA, 0, sizeof(Data_rxBuffA));
	return InitSPIBus();
}

/*
 * All this should do is read SPI data and store it.
 * Nothing else. Called from ISR at SAMPLE_RATE kHz.
 */
void getRawData()
{
	if(TotalReadTicks % (5000 / (flashFreq[0] * 2)) == 0 && flashFreq[0] > 0)
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_12);
	if(TotalReadTicks % (5000 / (flashFreq[1] * 2)) == 0 && flashFreq[1] > 0)
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	if(TotalReadTicks % (5000 / (flashFreq[2] * 2)) == 0 && flashFreq[2] > 0)
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_14);
	if(TotalReadTicks % (5000 / (flashFreq[3] * 2)) == 0 && flashFreq[3] > 0)
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_15);

	if(SYSTEM_STATE != SYS_STATE_RESET)
	{
		// Read gyro & accel data
		readIMU(&Data_rxBuffA[0]);
		accumulateIMU(&Data_rxBuffA[0]);

		// Decimate baro read at SAMPLE_RATE/100 Hz
		if(TotalReadTicks % 100 == 0)
		{
			readBARO(&Data_rxBuffA[BARO_START_RX]);
			accumulateBARO(&Data_rxBuffA[BARO_START_RX]);
		}

		// Run the filter on the current inertial measures @ SAMPLE_RATE
		if(SYS_READ_TIME(TotalReadTicks) > 0.1f)
		{
			MadgwickAHRSupdateIMU(
					xgyro_Ahrs * (PI / 180.0f),
					ygyro_Ahrs * (PI / 180.0f),
					zgyro_Ahrs * (PI / 180.0f),
					xaccl_Ahrs,
					yaccl_Ahrs,
					zaccl_Ahrs
			);
		}

		// Increment main counters at SAMPLE_RATE
		ReadTicks++;
		TotalReadTicks++;
		msgReadTicks++;

		// Flag the controller to process the accumulated data
		if(ReadTicks == (SAMPLE_RATE/CONTROL_RATE))
		{
			SYSTEM_STATE = SYS_STATE_PROCESSING;
		}
	}
}

/*
 * Run the data processor at F = SAMPLE_RATE/CONTROL_RATE  Hz
 */
void procRawData()
{
	ReadTicks = 0;

	procIMU();
	procBARO();

	// Compute roll/pitch/yaw angles from quaternions
	toEulerianAngle();
	roll_Ahrs 			= pitch_Madgwick;
	pitch_Ahrs 			= roll_Madgwick;
	yaw_Ahrs 			= yaw_Madgwick;

	// Tell system to run control loop
	SYSTEM_STATE = SYS_STATE_CONTROLLING;
}

/*
 * Prepare the SPI buses
 */
int InitSPIBus()
{
	int rc = 0;

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

	rc |= initIMU();
	rc |= initBARO();

	return rc;
}


int InitSamplingTimer()
{
	__HAL_RCC_TIM9_CLK_ENABLE();

	SamplingTimer.Init.Prescaler 	= 250; // 5 kHz = 100E6/((250)*(80)*(1))
    SamplingTimer.Init.CounterMode 	= TIM_COUNTERMODE_UP;
    SamplingTimer.Init.Period 		= 80; // 5 kHz = 100E6/((250)*(80)*(1))
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
