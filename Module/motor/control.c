#include "control.h"
#include "Wheel_motor.h"

//motor controling algorithm
uint8_t mode_select = 0;

PID right_main_motor_pid = {
    .kp = 0.5,
    .ki = 0.0,
    .kd = 0.0,
    .MaxIntegral = 0,
    .MaxOutput = 100
};

PID left_main_motor_pid = {
    .kp = 0.5,
    .ki = 0.0,
    .kd = 0.0,
    .MaxIntegral = 0,
    .MaxOutput = 100
};




void PID_Init(PID *pid, float P, float I, float D, float maxO, float maxI) {
    pid->kp = P;
    pid->ki = I;
    pid->kd = D;
    pid->MaxIntegral = maxI;
    pid->MaxOutput = maxO;

    pid->error = 0;
    pid->last_error = 0;
    pid->integral = 0;
    pid->output = 0;
}

void PID_calc(PID *pid, float target, float feedback) {
    pid->last_error = pid->error;
    pid->error = target - feedback;

    float pout = pid->kp * pid->error;
    float dout = pid->kd * (pid->error - pid->last_error);

    pid->integral += pid->error;

    // 限制积分项
    if (pid->integral > pid->MaxIntegral)
        pid->integral = pid->MaxIntegral;
    else if (pid->integral < -pid->MaxIntegral)
        pid->integral = -pid->MaxIntegral;

    float iout = pid->ki * pid->integral;

    pid->output = pout + iout + dout;

    // 限制输出
    if (pid->output > pid->MaxOutput)
        pid->output = pid->MaxOutput;
    else if (pid->output < -pid->MaxOutput)
        pid->output = -pid->MaxOutput;
}


//主要运动轮PID控制
void control_init(void){
    motor_init(&left_main_motor);
    motor_init(&right_main_motor);

}
void state_control(MAIN_MOTOR_TYPE *motor)   //状态控制与数据传递
{
    
}


void open_loop_straight_line(void){
    Get_state();
}

void open_loop_half_turn(void){

}

void open_loop_full_turn(void){

}



