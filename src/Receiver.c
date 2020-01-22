#include "Receiver.h"
#include "System.h"

static TIM_HandleTypeDef inputPWMTimer = { .Instance = TIM1 };
volatile char previous_state[4];
volatile uint32_t timer[4];
volatile uint16_t this_timer_value[4] = { 0, 0, 0, 0 };

volatile uint8_t chReady[4] = {0,0,0,0};

/*
 * Timer used for input PWM signals
 */
int InitReceiverTimer()
{
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_TIM1_CLK_ENABLE();

	inputPWMTimer.Instance = TIM1;
	inputPWMTimer.Init.Prescaler = inputPWMTimerPrescaler;
	inputPWMTimer.Init.CounterMode = TIM_COUNTERMODE_UP;
	inputPWMTimer.Init.Period = 0xFFFF;
	inputPWMTimer.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	inputPWMTimer.Init.RepetitionCounter = 0;

	if(HAL_TIM_IC_Init(&inputPWMTimer) != HAL_OK) return HAL_ERROR;

	TIM_IC_InitTypeDef sConfigIC = {0};
	sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
	sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
	sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
	sConfigIC.ICFilter = 0;

	if(HAL_TIM_IC_ConfigChannel(&inputPWMTimer, &sConfigIC, TIM_CHANNEL_1) != HAL_OK ||
	   HAL_TIM_IC_ConfigChannel(&inputPWMTimer, &sConfigIC, TIM_CHANNEL_2) != HAL_OK ||
	   HAL_TIM_IC_ConfigChannel(&inputPWMTimer, &sConfigIC, TIM_CHANNEL_3) != HAL_OK ||
	   HAL_TIM_IC_ConfigChannel(&inputPWMTimer, &sConfigIC, TIM_CHANNEL_4))
	{
		return HAL_ERROR;
	}

	if(HAL_TIM_IC_Start_IT(&inputPWMTimer, TIM_CHANNEL_1) != HAL_OK ||
	   HAL_TIM_IC_Start_IT(&inputPWMTimer, TIM_CHANNEL_2) != HAL_OK ||
	   HAL_TIM_IC_Start_IT(&inputPWMTimer, TIM_CHANNEL_3) != HAL_OK ||
	   HAL_TIM_IC_Start_IT(&inputPWMTimer, TIM_CHANNEL_4))
	{
		return HAL_ERROR;
	}

	return HAL_OK;
}

void TIM1_CC_IRQHandler(void)
{
	HAL_NVIC_ClearPendingIRQ(TIM1_CC_IRQn);
    HAL_TIM_IRQHandler(&inputPWMTimer);
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *tim_handle)
{
	if (tim_handle == &inputPWMTimer)
	{
		if(GPIOA->IDR & GPIO_PIN_8)
		{
			if(previous_state[CHANNEL_3_YAW] == LOW)
			{
				// Pin 8 going high
				previous_state[CHANNEL_3_YAW] = HIGH;
				timer[CHANNEL_3_YAW] = HAL_TIM_ReadCapturedValue(tim_handle, TIM_CHANNEL_1);
			}
		}
		else if(previous_state[CHANNEL_3_YAW] == HIGH)
		{
			previous_state[CHANNEL_3_YAW] = LOW;
			this_timer_value[CHANNEL_3_YAW] = HAL_TIM_ReadCapturedValue(tim_handle, TIM_CHANNEL_1) - timer[CHANNEL_3_YAW];
			chReady[CHANNEL_3_YAW] = 1;
		}


		if(GPIOA->IDR & GPIO_PIN_9)
		{
			if(previous_state[CHANNEL_4_THOT] == LOW)
			{
				// Pin 9 going high
				previous_state[CHANNEL_4_THOT] = HIGH;
				timer[CHANNEL_4_THOT] = HAL_TIM_ReadCapturedValue(tim_handle, TIM_CHANNEL_2);
			}
		}
		else if(previous_state[CHANNEL_4_THOT] == HIGH)
		{
			previous_state[CHANNEL_4_THOT] = LOW;
			this_timer_value[CHANNEL_4_THOT] = HAL_TIM_ReadCapturedValue(tim_handle, TIM_CHANNEL_2) - timer[CHANNEL_4_THOT];
			chReady[CHANNEL_4_THOT] = 1;
		}


		if(GPIOA->IDR & GPIO_PIN_10)
		{
			if(previous_state[CHANNEL_2_PITCH] == LOW)
			{
				// Pin 10 going high
				previous_state[CHANNEL_2_PITCH] = HIGH;
				timer[CHANNEL_2_PITCH] = HAL_TIM_ReadCapturedValue(tim_handle, TIM_CHANNEL_3);
			}
		}
		else if(previous_state[CHANNEL_2_PITCH] == HIGH)
		{
			previous_state[CHANNEL_2_PITCH] = LOW;
			this_timer_value[CHANNEL_2_PITCH] = HAL_TIM_ReadCapturedValue(tim_handle, TIM_CHANNEL_3) - timer[CHANNEL_2_PITCH];
			chReady[CHANNEL_2_PITCH] = 1;
		}


		if(GPIOA->IDR & GPIO_PIN_11)
		{
			if(previous_state[CHANNEL_1_ROLL] == LOW)
			{
				// Pin 11 going high
				previous_state[CHANNEL_1_ROLL] = HIGH;
				timer[CHANNEL_1_ROLL] = HAL_TIM_ReadCapturedValue(tim_handle, TIM_CHANNEL_4);
			}
		}
		else if(previous_state[CHANNEL_1_ROLL] == HIGH)
		{
			previous_state[CHANNEL_1_ROLL] = LOW;
			this_timer_value[CHANNEL_1_ROLL] = HAL_TIM_ReadCapturedValue(tim_handle, TIM_CHANNEL_4) - timer[CHANNEL_1_ROLL];
			chReady[CHANNEL_1_ROLL] = 1;
		}

		if(chReady[CHANNEL_1_ROLL] == 1 && chReady[CHANNEL_2_PITCH] == 1 && chReady[CHANNEL_3_YAW] == 1 && chReady[CHANNEL_4_THOT] == 1)
		{
			float tempChPW[4];
			for (int channel = CHANNEL_1_ROLL; channel < CHANNEL_NUM; channel++)
			{
				if(this_timer_value[channel] > 0 && this_timer_value[channel] < 0xFFFF)
					tempChPW[channel] = 1000000.0f/((100000000.0f / (float)(inputPWMTimerPrescaler + 1)) / (float)this_timer_value[channel]); // microseconds

				channelPulseWidth_us[channel] = (1.0f - 0.75f) * channelPulseWidth_us[channel] + (0.75f * tempChPW[channel]);
			}
		}
    }
}

void HAL_TIM_IC_MspInit(TIM_HandleTypeDef* htim_base)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(htim_base->Instance==TIM1)
  {
    /* Peripheral clock enable */
    __HAL_RCC_TIM1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**TIM1 GPIO Configuration
    PA8     ------> TIM1_CH1
    */
    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* TIM1 interrupt Init */
    HAL_NVIC_SetPriority(TIM1_CC_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM1_CC_IRQn);
  }
}
