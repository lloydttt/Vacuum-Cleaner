/******************************************************************************
 * @file    Wheel_motor.h
 * @brief   Header file for the wheel motor module.
 * @author  YanTao Lloyd Lai (lloydt@qq.com)
 * @version 1.0
 * @date    2025-4-14
 *
 * @copyright
 * Copyright (c) 2025 Vacuum Cleaner Project. All rights reserved.
 ******************************************************************************/
#pragma once

#include "main.h"
#include <stdio.h>
#define PI 3.14159265358979323846


typedef struct{
    uint8_t instance; // 0: left, 1: right
    uint8_t pwm_num;
    uint8_t drc;   //0 low 1 high, RIGHT HIGH FORWARD, LEFT LOW FORWARD
    float speed; // 速度
    uint16_t distance; // 距离
    
}MAIN_MOTOR_TYPE;


typedef struct {
    float linear;   // m/s
    float angular;  // rad/s
} Velocity2D;




void initVelocityFilters(void);
// PWM 速度转换，状态读取，控制接口
void motor_init(MAIN_MOTOR_TYPE *motor);

//一圈615     d = 64mm 

Velocity2D computeRobotVelocity(float v_left, float v_right, float wheel_base);
void Get_state(void);
void Motor_control(MAIN_MOTOR_TYPE *motor, float speed, int drc);

void sendOdomToQueue(Velocity2D *odom);
