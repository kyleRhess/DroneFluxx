#ifndef MAG_H_ /* include guard */
#define MAG_H_

#include "System.h"

SPI_Bus SPI_Bus_2;

extern float zmag;
extern float ymag;
extern float xmag;

void initMAG();
void readMAG(void);

#endif /* MAG_H_ */
