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
#include "transit_task.h"
Velocity2D _odom_data = {
    .linear = 0.0f,
    .angular = 0.0f
};
MAIN_MOTOR_TYPE left_main_motor = {    //motor2 GPIOC
    .instance = 0,
    .pwm_num = 99,
    .drc = 0,    // 1 增大减速往后   0 增大加速往前
    .speed = 0,
    .distance = 0
};
MAIN_MOTOR_TYPE right_main_motor = {   //motor1    GPIOB
    .instance = 1,
    .pwm_num = 1,
    .drc = 0,    //1 增大加速往后    0 增大减速往前
    .speed = 0,
    .distance = 0
};
uint32_t now_i;
volatile uint32_t count_left = 0;
volatile uint32_t count_right = 0;
volatile uint32_t last_count_left = 0;
volatile uint32_t last_count_right = 0;
static uint32_t right_last_tick = 0;
static uint32_t left_last_tick = 0;
volatile uint32_t count_left_tim = 0;
volatile uint32_t count_right_tim = 0;
volatile uint32_t delta_count = 0;
float aaa = 0;
float right_speed= 0;
float left_speed= 0;
float WHEEL_BASE = 0.183f; // Wheel base in meters
void motor_init(MAIN_MOTOR_TYPE *motor)
{
    if(motor->instance == 0){   //L
        if(motor->drc == 0)
        {
            HAL_GPIO_WritePin(GPIOC, MOTOR2_DRC_Pin, GPIO_PIN_RESET);
        }
        else if(motor->drc == 1)
        {

            HAL_GPIO_WritePin(GPIOC, MOTOR2_DRC_Pin, GPIO_PIN_SET);

        }
        HAL_TIM_PWM_Start_IT(&htim4, TIM_CHANNEL_3);
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);


    }else if(motor->instance == 1){    //R
        if(motor->drc == 0)
        {
            HAL_GPIO_WritePin(GPIOB, MOTOR1_DRC_Pin, GPIO_PIN_SET);
        }
        else if(motor->drc == 1)
        {

            HAL_GPIO_WritePin(GPIOB, MOTOR1_DRC_Pin, GPIO_PIN_RESET);
        }
        HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_2);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 100);


    }
    HAL_TIM_Base_Start_IT(&htim1);  // Enable TIM1 interrupt
    HAL_TIM_Base_Start_IT(&htim2);  // Enable TIM2 interrupt
    
}

Velocity2D computeRobotVelocity(float v_left, float v_right, float wheel_base) {
    Velocity2D v;
    v.linear =  (v_right + v_left) / 2.0f;
    v.angular = (v_right - v_left) / wheel_base;
    return v;
}
// void CheckMotorTimeout(void)
// {



//     now_i = HAL_GetTick();

//     // Right motor timeout check
//     if (count_right == last_count_right) {
//         if (now_i - right_last_tick > 1000) {  // 超过200ms无脉冲，认为停止
//             right_speed = 0.0f;
//             count_right = 0;
//             last_count_right = 0;
//         }
//     } 

//     // Left motor timeout check
//     if (count_left == last_count_left) {
//         if (now_i - left_last_tick > 1000) {
//             left_speed = 0.0f;
//             count_left = 0;
//             last_count_left = 0;
//         }
//     } 
// }
//左右轮获取速度
void Get_state(void)
{
    // CheckMotorTimeout();
    _odom_data = computeRobotVelocity(left_speed, right_speed, WHEEL_BASE);

    sendOdomToQueue(&_odom_data);
}

void Motor_control(MAIN_MOTOR_TYPE *motor,  float speed, int drc)
{
    if(motor->instance == 0){
        if(drc == 0)
        {
            HAL_GPIO_WritePin(GPIOC, MOTOR2_DRC_Pin, GPIO_PIN_RESET);     //ff
        }
        else if(drc == 1)
        {
            HAL_GPIO_WritePin(GPIOC, MOTOR2_DRC_Pin, GPIO_PIN_SET);      //bf
            speed = 100 - speed; // Reverse the speed for backward direction
        }
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, speed);

        
    }else if(motor->instance == 1){
        if(drc == 0)
        {
            HAL_GPIO_WritePin(GPIOB, MOTOR1_DRC_Pin, GPIO_PIN_SET);    //ff
            speed = 100 - speed; // Reverse the speed for forward direction
        }
        else if(drc == 1)
        {
            HAL_GPIO_WritePin(GPIOB, MOTOR1_DRC_Pin, GPIO_PIN_RESET);     //bf
        }
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, speed);
    }


}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
    if(htim->Instance == TIM1)
    {
        count_right_tim++;
        if(count_right_tim == 50){  // 0.25s
             delta_count = count_right - last_count_right;
            right_speed = (((float)delta_count / 615.0f) * 6.4f * PI / 250) * 1000.0f / 100.0f;  //  m/s
            last_count_right = count_right;
            count_right_tim = 0;
            right_main_motor.speed = right_speed; // Update the speed in the motor structure
        }



    }else if(htim->Instance == TIM2){
        count_left_tim++;
        if(count_left_tim == 50){
             delta_count = count_left - last_count_left;
            aaa = ((float)delta_count / 615.0f);
            left_speed = (((float)delta_count / 615.0f) * 6.4f * PI / 250) * 1000.0f / 100.0f;  //  m/s
            last_count_left = count_left;
            count_left_tim = 0;
            left_main_motor.speed = left_speed; // Update the speed in the motor structure
        }
        


    }
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM5)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == MOTOR1_ENCODER_IN1_Pin)
    {
        count_right++;
        // uint32_t now = HAL_GetTick(); 
        // if (count_right == 205)      //615
        // {
        //     uint32_t time_diff = now - right_last_tick; // Time difference in milliseconds
        //     right_speed = ((6.4f/3.0f) * PI / time_diff) * 1000.0f / 100.0f; // Speed in m per second  
        //     right_last_tick = now;
        //     count_right = 0;
        //     right_main_motor.speed = right_speed; // Update the speed in the motor structure
        //     // Print or store the calculated speed

        // }
        // last_count_right = count_right;
    }else if(GPIO_Pin == MOTOR2_ENCODER_IN1_Pin)
    {
        count_left++;
        // uint32_t now = HAL_GetTick(); 
        // if (count_left == 205)      
        // {
        //     uint32_t time_diff = now - left_last_tick; // Time difference in milliseconds
        //     left_speed = ((6.4f/3.0f) * PI / time_diff) * 1000.0f / 100.0f; // Speed in m per second  
        //     left_last_tick = now;
        //     count_left = 0;
        //     left_main_motor.speed = left_speed; // Update the speed in the motor structure
        //     // Print or store the calculated speed

        // }
        // last_count_left = count_left;
    }
}




void sendOdomToQueue(Velocity2D *odom) {
    UARTMessage msg;
    uint8_t *p = msg.data;

    *p++ = 0xA5;
    *p++ = 0x02;  // TYPE_ODOM
    *p++ = 8;

    memcpy(p, odom, 8);
    p += 8;

    uint8_t checksum = 0;
    for (int i = 0; i < 8; i++) {
        checksum += *((uint8_t *)odom + i);
    }
    *p = checksum;

    msg.length = 12;
    osMessageQueuePut(Getodom_UartQueueHandle(), &msg, 0, 0);
}





