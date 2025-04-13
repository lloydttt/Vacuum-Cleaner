#include "DT06.h"


GPIO_PinState ReadWiFiState(void) {
    return HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4);
}










