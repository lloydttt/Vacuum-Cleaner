#pragma once
#include "main.h"


uint8_t mode_select = 0;


typedef struct {
    float kp, ki, kd;
    float error, last_error;
    float integral, MaxIntegral;
    float output, MaxOutput;
} PID;

void PID_Init(PID *pid, float P, float I, float D, float maxO, float maxI);
void PID_calc(PID *pid, float target, float feedback);

//# Start Byte (0xA5) | Type (1=IMU, 2=Odom) | Length | Data | Checksum
