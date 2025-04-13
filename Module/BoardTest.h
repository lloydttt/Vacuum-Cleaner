#ifndef __BOARDTEST_H__
#define __BOARDTEST_H__

#include "gpio.h"

int flag = 0;
int index = 0;
void KBoardTest(){

      HAL_GPIO_WritePin(USER_LED_GPIO_Port, USER_LED_Pin, GPIO_PIN_SET);

      HAL_Delay(1000);
      HAL_GPIO_WritePin(USER_LED_GPIO_Port, USER_LED_Pin, GPIO_PIN_RESET);
      HAL_Delay(1000);
}

#endif