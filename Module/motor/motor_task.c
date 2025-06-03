#include "motor_task.h"
#include "control.h"

void motor_task_init(void){

    control_init();



}

/**
 * @brief 电机任务函数
 * @details 此函数用于执行电机相关的任务逻辑，通过调用状态控制函数实现电机的状态管理。
 * @note 此函数内部调用了 state_control 函数，用于处理电机的状态控制逻辑。
 */
void motor_task(void){

    state_control();


}





