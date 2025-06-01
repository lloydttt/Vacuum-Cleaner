#include "control.h"
#include "Wheel_motor.h"
#include "gpio.h"
#include "tim.h"
#include "usart.h"

//motor controling algorithm
uint8_t mode_select = 0;
uint8_t last_mode = 0;
uint8_t rx3_byte;    // USART1 接收缓冲
float cmd_linear_x = 0.3f;
float cmd_angular_z = 0.0f;
float ttt;
extern MAIN_MOTOR_TYPE left_main_motor;
extern MAIN_MOTOR_TYPE right_main_motor;
// 用于 USART3 0xA6 数据帧状态机
uint8_t rx_state = 0;
uint8_t rx_len = 0;
uint8_t rx_index = 0;
uint8_t rx_payload[8];

PID right_main_motor_pid = {
    .kp = -165,  //144
    .ki = 0.0,
    .kd = 0.0,
    .MaxIntegral = 100,
    .MaxOutput = 100
};

PID left_main_motor_pid = {
    .kp = -30,
    .ki = 0.01,
    .kd = 0.01,
    .MaxIntegral = 100,
    .MaxOutput = 100
};




void PID_Init(PID *pid, float P, float I, float D, float maxO, float maxI) {
    pid->kp = P;
    pid->ki = I;
    pid->kd = D;
    pid->MaxIntegral = maxI;
    pid->MaxOutput = maxO;

    pid->error = 0;
    pid->prev_error = 0;
    pid->past_error = 0;
    pid->last_error = 0;
    pid->integral = 0;
    pid->output = 0;
}

void PID_Incremental_Calc(PID *pid, float target, float feedback) {
    pid->prev_error = pid->last_error;
    pid->last_error = pid->error;
    pid->error = target - feedback;

    float delta_output = pid->kp * (pid->error - pid->last_error)
                       + pid->ki * pid->error
                       + pid->kd * (pid->error - 2.0f * pid->last_error + pid->prev_error);

    pid->output += delta_output;

    // 限幅处理
    if (pid->output > pid->MaxOutput)
        pid->output = pid->MaxOutput;
    else if (pid->output < 0)
        pid->output = -pid->output;
}



//主要运动轮PID控制
void control_init(void){
    initVelocityFilters();
    motor_init(&left_main_motor);
    motor_init(&right_main_motor);
    HAL_UART_Receive_IT(&huart3, &rx3_byte, 1);
}
void state_control()   //状态控制与数据传递
{
    mode_check(cmd_linear_x, cmd_angular_z);
    if(mode_select == 0){
        if(last_mode == 0){
            //停止
            Motor_control(&left_main_motor, 0, 0);
            Motor_control(&right_main_motor, 0, 0);
            Get_state();

        }else if (last_mode == 1){
            open_loop_straight_line_forword_STOP();
        }else if (last_mode == 2){
            open_loop_straight_line_backword_STOP();
        }else if (last_mode == 3){
            open_loop_point_turn_left_STOP();
        }else if (last_mode == 4){
            open_loop_point_turn_right_STOP();
        }
        
    }else if(mode_select == 1){
        open_loop_straight_line_forword();
        last_mode = 1;  
    }else if(mode_select == 2){
        open_loop_straight_line_backword();
        last_mode = 2;  
    }else if(mode_select == 4){
        open_loop_point_turn_left();
        last_mode = 4;  
    }else if(mode_select == 3){
        open_loop_point_turn_right();
        last_mode = 3;  
    }

    // open_loop_straight_line_forword();
    // open_loop_straight_line_backword();
    // open_loop_point_turn_left();
    // open_loop_point_turn_right();
    // motor_ttt();
}


void open_loop_straight_line_forword(void){
    left_main_motor.drc = 0;  // forward
    right_main_motor.drc = 0; // forward
    Motor_control(&left_main_motor, 50+20, left_main_motor.drc);
    Motor_control(&right_main_motor, 45, right_main_motor.drc);
    Get_state();
}

void open_loop_straight_line_forword_STOP(void){
    Motor_control(&left_main_motor, 0, 0);
    Motor_control(&right_main_motor, 0, 0);
    Get_state();

}
void open_loop_straight_line_backword(void){
    left_main_motor.drc = 1;  // backward
    right_main_motor.drc = 1; // backward
    Motor_control(&left_main_motor, 38, left_main_motor.drc);
    Motor_control(&right_main_motor, 50+20.2, right_main_motor.drc);
    Get_state();
}    //   0.5米误差不超过5mm
void open_loop_straight_line_backword_STOP(void){
    Motor_control(&left_main_motor, 0, 1);
    Motor_control(&right_main_motor, 0, 1);
    Get_state();


}
void open_loop_half_turn(void){

}

void open_loop_full_turn(void){

}


void open_loop_point_turn_left(void){
    left_main_motor.drc = 1;  
    right_main_motor.drc = 0;
    Motor_control(&left_main_motor, 25, left_main_motor.drc);
    Motor_control(&right_main_motor, 25, right_main_motor.drc);
    Get_state();



}
void open_loop_point_turn_left_STOP(void){
    Motor_control(&left_main_motor, 0, 1);
    Motor_control(&right_main_motor, 0, 0);
    Get_state();

}

void open_loop_point_turn_right(void){
    left_main_motor.drc = 0;  
    right_main_motor.drc = 1;
    Motor_control(&left_main_motor, 50+15, left_main_motor.drc);
    Motor_control(&right_main_motor, 50+15, right_main_motor.drc);
    Get_state();


}
void open_loop_point_turn_right_STOP(void){
    Motor_control(&left_main_motor, 0, 0);
    Motor_control(&right_main_motor, 0, 1);
    Get_state();


}
void mode_check(float _cmd_linear_x, float _cmd_angular_z){
    if(_cmd_linear_x == 0 && _cmd_angular_z == 0){
        mode_select = 0;
    }else if(_cmd_linear_x == 1){
        mode_select = 1;  //ff
    }else if(_cmd_linear_x == -1){
        mode_select = 2;  //bf
    }else if(_cmd_angular_z == 1){
        mode_select = 3;  //left turn 
    }else if(_cmd_angular_z == -1){
        mode_select = 4;  //right turn
    }

}


void motor_ttt(void){

    PID_Incremental_Calc(&right_main_motor_pid, 0.12, right_main_motor.speed);
    Motor_control(&right_main_motor, right_main_motor_pid.output, right_main_motor.drc);
    PID_Incremental_Calc(&left_main_motor_pid, 0.12, left_main_motor.speed);
    float ff_output = 60;
    Motor_control(&left_main_motor, ff_output+left_main_motor_pid.output, 0);

}