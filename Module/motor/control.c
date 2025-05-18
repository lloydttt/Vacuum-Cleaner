#include "control.h"


//motor controling algorithm


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
