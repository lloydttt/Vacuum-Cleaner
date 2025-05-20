#pragma once
#include "main.h"
#include "Wheel_motor.h"




typedef struct {
    float kp, ki, kd;
    float error, last_error;
    float integral, MaxIntegral;
    float output, MaxOutput;
} PID;

void PID_Init(PID *pid, float P, float I, float D, float maxO, float maxI);
void PID_calc(PID *pid, float target, float feedback);

//# Start Byte (0xA5) | Type (1=IMU, 2=Odom) | Length | Data | Checksum
void control_init(void);
void state_control();
//@todo: open loop straight line, half turn, full turn
void open_loop_straight_line_forword(void);
void open_loop_straight_line_backword(void);
void open_loop_half_turn(void);
void open_loop_full_turn(void);
void open_loop_point_turn_left(void);
void open_loop_point_turn_right(void);
void mode_check(float _cmd_linear_x, float _cmd_angular_z);