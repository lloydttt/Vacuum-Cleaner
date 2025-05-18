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

#ifndef __WHEEL_MOTOR_H__
#define __WHEEL_MOTOR_H__

// PWM 速度转换，状态读取，控制接口

void PWM_genrate(uint16_t speed);
void Get_state(void);
void Motor_control(void);


#endif