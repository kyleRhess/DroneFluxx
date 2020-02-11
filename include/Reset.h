#ifndef RESET_H_ /* include guard */
#define RESET_H_

#include <stdio.h>

#define RST_PIN				GPIO_PIN_1
#define MILLIS				500

int Reset_Init(void);
int Reset_Check(void);

#endif /* RESET_H_ */
