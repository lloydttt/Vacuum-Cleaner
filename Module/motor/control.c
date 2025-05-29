#include "control.h"
#include "Wheel_motor.h"
#include "gpio.h"
#include "tim.h"
#include "usart.h"

//motor controling algorithm
uint8_t mode_select = 2;
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
    .kp = 600,
    .ki = 0.0,
    .kd = 0.0,
    .MaxIntegral = 100,
    .MaxOutput = 100
};

PID left_main_motor_pid = {
    .kp = 0.5,
    .ki = 0.0,
    .kd = 0.0,
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

void PID_calc(PID *pid, float target, float feedback){
    pid->past_error = pid->last_error;
    pid->last_error = pid->error;
    pid->error = (target - feedback);

    float pout = pid->error * pid->kp;
    float dout = (pid->error - 2*pid->last_error + pid->past_error) * pid->kd;

    pid->integral += pid->error;
    if(pid->integral > pid->MaxIntegral)
        pid->integral = pid->MaxIntegral;
    else if (pid->integral < -pid->MaxIntegral)
        pid->integral = -pid->MaxIntegral;

    float iout = pid->integral * pid->ki;

    pid->output = pout + iout + dout;

    if (pid->output > pid->MaxOutput)
        pid->output = pid->MaxOutput;
    else if(pid->output < -pid->MaxOutput)
        pid->output = -pid->MaxOutput;
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
    else if (pid->output < -pid->MaxOutput)
        pid->output = -pid->MaxOutput;
}



//主要运动轮PID控制
void control_init(void){
    motor_init(&left_main_motor);
    motor_init(&right_main_motor);
    HAL_UART_Receive_IT(&huart3, &rx3_byte, 1);
}
void state_control()   //状态控制与数据传递
{
    // mode_check(cmd_linear_x, cmd_angular_z);
    // if(mode_select == 0){
    //     if(last_mode == 0){
    //         //停止
    //         HAL_GPIO_WritePin(GPIOC, MOTOR2_DRC_Pin, GPIO_PIN_SET);
    //         HAL_GPIO_WritePin(GPIOB, MOTOR1_DRC_Pin, GPIO_PIN_RESET);
    //         __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 90);
    //         __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 10);
    //         CheckMotorTimeout();
    //         Get_state();

    //     }else if (last_mode == 1){
    //         open_loop_straight_line_forword_STOP();
    //     }else if (last_mode == 2){
    //         open_loop_straight_line_backword_STOP();
    //     }else if (last_mode == 3){
    //         open_loop_point_turn_left_STOP();
    //     }else if (last_mode == 4){
    //         open_loop_point_turn_right_STOP();
    //     }
        
    // }else if(mode_select == 1){
    //     open_loop_straight_line_forword();
    //     last_mode = 1;  
    // }else if(mode_select == 2){
    //     open_loop_straight_line_backword();
    //     last_mode = 2;  
    // }else if(mode_select == 3){
    //     open_loop_point_turn_left();
    //     last_mode = 3;  
    // }else if(mode_select == 4){
    //     open_loop_point_turn_right();
    //     last_mode = 4;  
    // }

    open_loop_straight_line_forword();
    // motor_ttt();
}


void open_loop_straight_line_forword(void){
    HAL_GPIO_WritePin(GPIOB, MOTOR1_DRC_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, MOTOR2_DRC_Pin, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 45);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 75);
    Get_state();
}

void open_loop_straight_line_forword_STOP(void){
    HAL_GPIO_WritePin(GPIOC, MOTOR2_DRC_Pin, GPIO_PIN_RESET);  
    HAL_GPIO_WritePin(GPIOB, MOTOR1_DRC_Pin, GPIO_PIN_SET);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 100);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
    CheckMotorTimeout();
    Get_state();

}
void open_loop_straight_line_backword(void){
    HAL_GPIO_WritePin(GPIOB, MOTOR1_DRC_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, MOTOR2_DRC_Pin, GPIO_PIN_SET);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 70);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 60);
    Get_state();
}
void open_loop_straight_line_backword_STOP(void){
    HAL_GPIO_WritePin(GPIOC, MOTOR2_DRC_Pin, GPIO_PIN_SET);  
    HAL_GPIO_WritePin(GPIOB, MOTOR1_DRC_Pin, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 100);
    CheckMotorTimeout();
    Get_state();


}
void open_loop_half_turn(void){

}

void open_loop_full_turn(void){

}


void open_loop_point_turn_left(void){
    HAL_GPIO_WritePin(GPIOB, MOTOR1_DRC_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, MOTOR2_DRC_Pin, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 70);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 70);
    Get_state();



}
void open_loop_point_turn_left_STOP(void){
    HAL_GPIO_WritePin(GPIOC, MOTOR1_DRC_Pin, GPIO_PIN_SET);  
    HAL_GPIO_WritePin(GPIOB, MOTOR2_DRC_Pin, GPIO_PIN_SET);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
    CheckMotorTimeout();
    Get_state();

}

void open_loop_point_turn_right(void){
    HAL_GPIO_WritePin(GPIOB, MOTOR1_DRC_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, MOTOR2_DRC_Pin, GPIO_PIN_SET);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 70);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 70);
    Get_state();


}
void open_loop_point_turn_right_STOP(void){

    HAL_GPIO_WritePin(GPIOC, MOTOR1_DRC_Pin, GPIO_PIN_RESET);  
    HAL_GPIO_WritePin(GPIOB, MOTOR2_DRC_Pin, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 100);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 100);
    CheckMotorTimeout();
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
    // ttt = right_main_motor.speed;
    // PID_calc(&right_main_motor_pid, cmd_linear_x, right_main_motor.speed);
    // Motor_control(&left_main_motor, 0, 1);
    // HAL_Delay(1500);
    // Motor_control(&left_main_motor, 50, 1);
    // HAL_Delay(1500);
    // Motor_control(&left_main_motor, 100, 1);
    // HAL_Delay(1500);
    // Motor_control(&left_main_motor, 0, 0);
    // HAL_Delay(1500);
    // Motor_control(&left_main_motor, 50, 0);  
    // HAL_Delay(1500);
    // Motor_control(&left_main_motor, 100, 0);
    // HAL_Delay(1500);
    PID_Incremental_Calc(&right_main_motor_pid, 0.10, right_main_motor.speed);
    Motor_control(&right_main_motor, right_main_motor_pid.output, right_main_motor.drc);
    // CheckMotorTimeout();

    // Motor_control(&right_main_motor, right_main_motor_pid.output, right_main_motor.drc);

}