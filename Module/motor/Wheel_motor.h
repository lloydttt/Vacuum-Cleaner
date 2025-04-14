/******************************************************************************
 * @file    Wheel_motor.h
 * @brief   Header file for the wheel motor module.
 * @author  YanTao Lloyd Lai (lloydt@qq.com)
 * @version 1.0
 * @date    2025-4-14
 *
 * @copyright
 * Copyright (c) 2025 Vacuum Cleaner Project. All rights reserved.
 * 
 * This software is the property of its respective owners and is protected
 * under copyright law. Unauthorized use, duplication, or distribution of
 * this software is strictly prohibited.
 ******************************************************************************/
#pragma once

#ifndef __WHEEL_MOTOR_H__
#define __WHEEL_MOTOR_H__

uint8_t Direction;
uint16_t counter;
uint16_t enc1 = 0,enc1_old = 0;
int16_t enc2 = 0;
int32_t enc;





#endif