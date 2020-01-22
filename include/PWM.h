#ifndef PWM_H_ /* include guard */
#define PWM_H_

#include <string.h>
#include <math.h>
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_conf_template.h"
#include "stm32f4xx_hal_tim.h"
#include "stm32f4xx_hal_tim_ex.h"

#define CLOCK_CYCLES_PER_SECOND  100000000
#define MAX_RELOAD               0xFFFF
#define PWM_FREQ 				 30000
#define PWM_STEPS				 367 // ratio = (PWM_FREQ = 1000 / PWM_STEPS = 11000)
#define COUNTERFREQ 			(PWM_FREQ * PWM_STEPS)
#define PULSE_NS_PER_CNT		 88
#define CNTS_FROM_US(xxx)		((xxx * 1000) / PULSE_NS_PER_CNT)

typedef struct PWM_OUTPUT
{
	TIM_HandleTypeDef 	timer;
	float 				dutyCycle;
	uint32_t 			frequency;
	uint16_t 			numChannels;
	uint32_t 			Channel;
	TIM_TypeDef*		TIM;
} PWM_Out;

extern TIM_HandleTypeDef timer_PWM;

TIM_HandleTypeDef Initialize_PWM(PWM_Out * PWMType);
void PWM_adjust_DutyCycle(TIM_HandleTypeDef * pwmHandle, uint32_t Channel, float dutyCycle);
void PWM_adjust_PulseWidth(TIM_HandleTypeDef * pwmHandle, uint32_t Channel, float pulseWidth_us);
void PWM_adjust_Frequency(TIM_HandleTypeDef * pwmHandle, uint32_t Channel, uint32_t newFreq);
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef* htim_pwm);

#endif /* PWM_H_ */
