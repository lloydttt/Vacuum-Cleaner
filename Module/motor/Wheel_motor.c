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

Velocity2D _odom_data = {
    .linear = 0,
    .angular = 0
};

volatile uint32_t count_left = 0;
volatile uint32_t count_right = 0;
static uint32_t right_last_tick = 0;
static uint32_t left_last_tick = 0;
float right_speed= 0;
float left_speed= 0;
float WHEEL_BASE = 0.183f; // Wheel base in meters
void motor_init(MAIN_MOTOR_TYPE *motor)
{
    if(motor->instance == 1){
        if(motor->drc == 0)
        {
            HAL_GPIO_WritePin(GPIOC, MOTOR1_DRC_Pin, GPIO_PIN_RESET);
        }
        else if(motor->drc == 1)
        {

            HAL_GPIO_WritePin(GPIOC, MOTOR1_DRC_Pin, GPIO_PIN_SET);

        }
        HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_2);
    }else if(motor->instance == 0){
        if(motor->drc == 0)
        {
            HAL_GPIO_WritePin(GPIOB, MOTOR2_DRC_Pin, GPIO_PIN_SET);
        }
        else if(motor->drc == 1)
        {

            HAL_GPIO_WritePin(GPIOB, MOTOR2_DRC_Pin, GPIO_PIN_RESET);
        }
        HAL_TIM_PWM_Start_IT(&htim4, TIM_CHANNEL_3);
    }
    
}

Velocity2D computeRobotVelocity(float v_left, float v_right, float wheel_base) {
    Velocity2D v;
    v.linear =  (v_right + v_left) / 2.0f;
    v.angular = (v_right - v_left) / wheel_base;
    return v;
}
//左右轮获取速度
void Get_state(void)
{
    _odom_data = computeRobotVelocity(left_speed, right_speed, WHEEL_BASE);


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
        if (count_right == 615)      
        {
            uint32_t time_diff = now - right_last_tick; // Time difference in milliseconds
            right_speed = (6.4f * PI / time_diff) * 1000.0f / 100.0f; // Speed in m per second  
            right_last_tick = now;
            count_right = 0;

            // Print or store the calculated speed

        }
        count_right++;
    }else if(GPIO_Pin == MOTOR2_ENCODER_IN1_Pin)
    {
        uint32_t now = HAL_GetTick(); 
        if (count_left == 615)      
        {
            uint32_t time_diff = now - left_last_tick; // Time difference in milliseconds
            left_speed = (6.4f * PI / time_diff) * 1000.0f / 100.0f; // Speed in m per second  
            left_last_tick = now;
            count_left = 0;

            // Print or store the calculated speed

        }
        count_left++;
    }
}









