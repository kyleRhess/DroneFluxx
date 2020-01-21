#ifndef RECEIVER_H_ /* include guard */
#define RECEIVER_H_

#include <stdio.h>
#include "stm32f4xx.h"

#define sSteps 20000
#define sFreq  500
#define cFreq  (sFreq * sSteps)
#define inputPWMTimerPrescaler ((100000000 / cFreq) - 1)
#define inputPWMTimerArr (sSteps - 1)

volatile char previous_state[4];
volatile uint32_t timer[4];
volatile uint16_t this_timer_value[4];

int InitReceiverTimer(void);

#endif /* RECEIVER_H_ */
