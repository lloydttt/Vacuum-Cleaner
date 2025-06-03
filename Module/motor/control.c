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
    .kp = 165,  //144
    .ki = 0.0,
    .kd = 0.0,
    .MaxIntegral = 100,
    .MaxOutput = 100
};

PID left_main_motor_pid = {
    .kp = 30,
    .ki = 0.01,
    .kd = 0.01,
    .MaxIntegral = 100,
    .MaxOutput = 100
};




/**
 * @brief 初始化 PID 控制器的函数
 * 
 * @param pid 指向 PID 控制器结构体的指针
 * @param P 比例系数
 * @param I 积分系数
 * @param D 微分系数
 * @param maxO 输出的最大值
 * @param maxI 积分的最大值
 * 
 * @note 该函数用于初始化 PID 控制器的各项参数，包括比例、积分、微分系数，
 *       以及积分和输出的最大值，并重置误差和积分值。
 */
void PID_Init(PID *pid, float P, float I, float D, float maxO, float maxI) {
    pid->kp = P;               // 设置比例系数
    pid->ki = I;               // 设置积分系数
    pid->kd = D;               // 设置微分系数
    pid->MaxIntegral = maxI;   // 设置积分的最大值
    pid->MaxOutput = maxO;     // 设置输出的最大值

    pid->error = 0;            // 初始化当前误差
    pid->prev_error = 0;       // 初始化前一次误差
    pid->past_error = 0;       // 初始化更早的误差
    pid->last_error = 0;       // 初始化最后一次误差
    pid->integral = 0;         // 初始化积分值
    pid->output = 0;           // 初始化输出值
}

/**
 * @brief 增量式 PID 计算函数
 * 
 * @param pid 指向 PID 结构体的指针
 * @param target 目标值
 * @param feedback 当前反馈值
 * 
 * @note 该函数基于目标值与反馈值的差值计算增量式 PID 输出。
 *       它更新 PID 结构体中的误差项，并增量调整输出值。
 *       输出值被限制在 PID 结构体定义的最大允许值范围内。
 *       如果输出值为负，则将其取反以确保其为正值。
 */
void PID_Incremental_Calc(PID *pid, float target, float feedback) {
    pid->prev_error = pid->last_error;
    pid->last_error = pid->error;
    pid->error = target - feedback;

    float delta_output = pid->kp * (pid->last_error - pid->error)
                       + pid->ki * pid->error
                       + pid->kd * (pid->error - 2.0f * pid->last_error + pid->prev_error);

    pid->output += delta_output;

    // 限幅处理
    if (pid->output > pid->MaxOutput)
        pid->output = pid->MaxOutput;
    else if (pid->output < 0)
        pid->output = -pid->output;
}



/**
 * @brief 控制模块初始化函数
 * @details 初始化速度滤波器，初始化左右主电机，并开启UART接收中断。
 * @note 此函数在系统启动时调用，用于完成控制模块的基本初始化。
 */
void control_init(void){
    initVelocityFilters();
    motor_init(&left_main_motor);
    motor_init(&right_main_motor);
    HAL_UART_Receive_IT(&huart3, &rx3_byte, 1);
}
/**
 * @brief 状态控制与数据传递函数
 * @details 根据当前模式选择 (mode_select) 和上一次模式 (last_mode)，
 *          控制电机的运行状态，包括停止、前进、后退、左转和右转。
 *          同时调用相关函数进行模式检测和状态获取。或者闭环控制运行
 * 
 * @note 该函数通过 mode_check 函数检测当前模式，并根据 mode_select 的值
 *       执行对应的电机控制逻辑。函数还会更新 last_mode 以记录上一次的模式。
 * 
 * @param 无
 * @return 无
 */
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

    // motor_closeLoop_control();
}


/**
 * @brief 开环直线前进控制函数
 * @details 此函数通过设置左右主电机的方向和速度，实现开环控制下的直线前进。
 * @note 左主电机的速度为 50 + 28，右主电机的速度为 60。
 *       调用 Motor_control 函数控制电机，并调用 Get_state 函数获取当前状态。
 */
void open_loop_straight_line_forword(void){
    left_main_motor.drc = 0;  // forward
    right_main_motor.drc = 0; // forward
    Motor_control(&left_main_motor, 50+28, left_main_motor.drc);
    Motor_control(&right_main_motor, 60, right_main_motor.drc);
    Get_state();
}

void open_loop_straight_line_forword_STOP(void){
    Motor_control(&left_main_motor, 0, 0);
    Motor_control(&right_main_motor, 0, 0);
    Get_state();

}
/**
 * @brief 开环控制机器人以直线向后运动
 * 
 * @note 此函数设置左右主电机的方向为后退，并分别以不同的占空比控制电机速度。
 *       左主电机的占空比为55，右主电机的占空比为80（50+30）。
 *       调用Motor_control函数实现电机控制，并通过Get_state函数获取当前状态。
 * 
 * @details 该函数用于实现机器人以直线向后运动，适用于对运动精度要求较高的场景。
 *          设计目标为在0.5米的运动距离内误差不超过5毫米。
 */
void open_loop_straight_line_backword(void){
    left_main_motor.drc = 1;  // backward
    right_main_motor.drc = 1; // backward
    Motor_control(&left_main_motor, 55, left_main_motor.drc);
    Motor_control(&right_main_motor, 50+30, right_main_motor.drc);
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


/**
 * @brief 左转点转的开环控制函数
 * @note 此函数通过设置左右主电机的方向和速度，实现机器人左转点转的功能。
 *       左主电机设置为正转，右主电机设置为反转，速度均为25。
 *       调用 Motor_control 函数控制电机，并通过 Get_state 函数获取当前状态。
 */
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
/**
 * @brief 开环右转点转控制函数
 * @details 此函数通过设置左右主电机的方向和速度，实现开环控制下的右转点转动作。
 * @note 调用此函数时，左主电机方向设置为正转，右主电机方向设置为反转，
 *       并通过 Motor_control 函数设置两个电机的速度为 65（50+15）。
 *       最后调用 Get_state 函数获取当前状态。
 */

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


/**
 * @brief 电机闭环控制函数
 * @details 该函数实现了左右主电机的闭环控制，通过增量式PID算法计算控制量，
 *          并调用电机控制函数对电机进行驱动。
 * 
 * @note 右主电机使用PID控制，左主电机在PID控制基础上增加前馈控制量。
 */
void motor_closeLoop_control(void){
    if(!avoid_control()){
        PID_Incremental_Calc(&right_main_motor_pid, cmd_linear_x, right_main_motor.speed);
        Motor_control(&right_main_motor, right_main_motor_pid.output, right_main_motor.drc);
        PID_Incremental_Calc(&left_main_motor_pid, cmd_linear_x, left_main_motor.speed);
        float ff_output = 60;
        Motor_control(&left_main_motor, ff_output+left_main_motor_pid.output, 0);
    }

    
}

/**
 * @brief 防坠落控制函数
 * @details 该函数通过检测两个信号引脚的状态，判断是否需要进行防坠落操作。
 *          如果检测到任一信号引脚为低电平，则触发相应的防坠落动作：
 *          - 如果 SIGNAL1 引脚为低电平，执行右转操作。
 *          - 如果 SIGNAL2 引脚为低电平，执行左转操作。
 * 
 * @return int
 *         - 返回 1 表示已执行防坠落操作。
 *         - 返回 0 表示无需防坠落操作。
 * 
 * @note 该函数依赖 HAL_GPIO_ReadPin 函数读取引脚状态，并调用相应的电机控制函数实现防坠落动作。
 */
int avoid_control(void){
    if(HAL_GPIO_ReadPin(SIGNAL1_GPIO_Port, SIGNAL1_Pin) == GPIO_PIN_RESET || HAL_GPIO_ReadPin(SIGNAL2_GPIO_Port, SIGNAL2_Pin == GPIO_PIN_RESET)){
        
        if(HAL_GPIO_ReadPin(SIGNAL1_GPIO_Port, SIGNAL1_Pin) == GPIO_PIN_RESET)
        {
            open_loop_point_turn_right();
        }
        else if (HAL_GPIO_ReadPin(SIGNAL2_GPIO_Port, SIGNAL2_Pin) == GPIO_PIN_RESET)
        {
            open_loop_point_turn_left();
        }
        return 1; // Avoidance action taken
    }
    return 0; // No avoidance action needed
}