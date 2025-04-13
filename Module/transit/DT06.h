#ifndef __DT06_H__
#define __DT06_H__

#include "gpio.h"

void DT06_Init(void);

GPIO_PinState ReadWiFiState(void);


#endif // __DT06_H__