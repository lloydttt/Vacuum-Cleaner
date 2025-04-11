#ifndef __BOARDTEST_H__
#define __BOARDTEST_H__

#include "gpio.h"

int flag = 0;
int index = 0;
void KBoardTest(){
    if (flag == 0){
        HAL_GPIO_WritePin(USER_LED_GPIO_Port, USER_LED_Pin, GPIO_PIN_SET);

    }
    else{
        HAL_GPIO_WritePin(USER_LED_GPIO_Port, USER_LED_Pin, GPIO_PIN_RESET);

    }
    if (index > 1000){
        flag = !flag;
        index = 0;
    }
    index++;

}

#endif