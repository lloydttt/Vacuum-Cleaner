#include "control.h"
#include "Wheel_motor.h"
#include "gpio.h"
#include "tim.h"
#include "usart.h"

//motor controling algorithm
uint8_t mode_select = 0;
uint8_t rx3_byte;    // USART1 接收缓冲
float cmd_linear_x = 0.0f;
float cmd_angular_z = 0.0f;

// 用于 USART3 0xA6 数据帧状态机
uint8_t rx_state = 0;
uint8_t rx_len = 0;
uint8_t rx_index = 0;
uint8_t rx_payload[8];

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
    HAL_UART_Receive_IT(&huart3, &rx3_byte, 1);
}
void state_control()   //状态控制与数据传递
{
    mode_check(cmd_linear_x, cmd_angular_z);
    if(mode_select == 0){
        //停止
        HAL_GPIO_WritePin(GPIOC, MOTOR1_DRC_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, MOTOR2_DRC_Pin, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 99);
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 1);
        
    }else if(mode_select == 1){
        open_loop_straight_line_forword();
    }else if(mode_select == 2){
        open_loop_straight_line_backword();
    }else if(mode_select == 3){
        open_loop_point_turn_left();
    }else if(mode_select == 4){
        open_loop_point_turn_right();
    }

}


void open_loop_straight_line_forword(void){
    // HAL_GPIO_WritePin(GPIOC, MOTOR1_DRC_Pin, GPIO_PIN_SET);
    // HAL_GPIO_WritePin(GPIOB, MOTOR2_DRC_Pin, GPIO_PIN_RESET);
    // __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 50);
    // __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 50);
    HAL_GPIO_WritePin(GPIOB, MOTOR1_DRC_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, MOTOR2_DRC_Pin, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 30);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 80);
    Get_state();
}
void open_loop_straight_line_backword(void){
    HAL_GPIO_WritePin(GPIOB, MOTOR1_DRC_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, MOTOR2_DRC_Pin, GPIO_PIN_SET);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 80);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 50);
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
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 80);



}

void open_loop_point_turn_right(void){
    HAL_GPIO_WritePin(GPIOB, MOTOR1_DRC_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, MOTOR2_DRC_Pin, GPIO_PIN_SET);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 30);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 70);


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