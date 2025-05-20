/******************************************************************************
 * @file    Wheel_motor.c
 * @brief   Motor of vaccum cleaner dirving source code.
 * @author  YanTao Lloyd Lai (lloydt@qq.com)
 * @version 1.0
 * @date    2025-4-14
 *
 * @copyright
 * Copyright (c) 2025 Vacuum Cleaner Project. All rights reserved.
 ******************************************************************************/
#include "Wheel_motor.h"
#include "tim.h"

MAIN_MOTOR_TYPE left_main_motor = {
    .instance = 0,
    .pwm_num = 99,
    .drc = 0,
    .speed = 0,
    .distance = 0
};
MAIN_MOTOR_TYPE right_main_motor = {
    .instance = 1,
    .pwm_num = 1,
    .drc = 1,
    .speed = 0,
    .distance = 0
};

uint32_t count = 0;
static uint32_t last_tick = 0;
float speed= 0;
void motor_init(MAIN_MOTOR_TYPE *motor)
{
    if(motor->instance == 0){
        if(motor->drc == 0)
        {
            HAL_GPIO_WritePin(GPIOC, MOTOR2_DRC_Pin, GPIO_PIN_RESET);
        }
        else if(motor->drc == 1)
        {

            HAL_GPIO_WritePin(GPIOC, MOTOR2_DRC_Pin, GPIO_PIN_SET);

        }
    }else if(motor->instance == 1){
        if(motor->drc == 0)
        {
            HAL_GPIO_WritePin(GPIOB, MOTOR1_DRC_Pin, GPIO_PIN_SET);
        }
        else if(motor->drc == 1)
        {

            HAL_GPIO_WritePin(GPIOB, MOTOR1_DRC_Pin, GPIO_PIN_RESET);
        }
    }
    HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_2);
}

//左右轮获取速度
void Get_state(void)
{





}

void Motor_control(MAIN_MOTOR_TYPE *motor)
{
    if(motor->instance == 0){


        
    }else if(motor->instance == 1){

    }


}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == MOTOR1_ENCODER_IN1_Pin)
    {
        uint32_t now = HAL_GetTick(); 
        if (count == 615)      
        {
            uint32_t time_diff = now - last_tick; // Time difference in milliseconds
            speed = (6.4f * PI / time_diff) * 1000.0f; // Speed in cm per second  
            last_tick = now;
            count = 0;

            // Print or store the calculated speed

        }
        count++;
    }
}









