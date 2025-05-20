#include "motor_task.h"
#include "control.h"

void motor_task_init(void){

    control_init();



}

void motor_task(void){

    open_loop_straight_line();


}





