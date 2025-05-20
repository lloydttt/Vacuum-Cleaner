#include "robot_task.h"
#include "ins_task.h"
#include "motor_task.h"
#include "transit_task.h"

osThreadId_t insTaskHandle;
osThreadId_t motorTaskHandle;
osThreadId_t transTaskHandle;
void OSTaskInit()
{
    const osThreadAttr_t insTaskAttr = {
        .name = "insTask",
        .priority = osPriorityAboveNormal,
        .stack_size = 1024
    };
    const osThreadAttr_t transTaskAttr = {
        .name = "transitTask",
        .priority = osPriorityNormal,
        .stack_size = 1024
    };
    const osThreadAttr_t motorTaskAttr = {
        .name = "motorTask",
        .priority = osPriorityNormal,
        .stack_size = 1024
    };

    insTaskHandle = osThreadNew(StartINSTASK, NULL, &insTaskAttr);
    motorTaskHandle = osThreadNew(StartMOTORTASK, NULL, &motorTaskAttr);
    transTaskHandle = osThreadNew(StartTRANSTASK, NULL, &transTaskAttr);

}

__attribute__((noreturn)) void StartINSTASK(void *argument)
{
    ins_init();

    for (;;)
    {
        ins_task();
        osDelay(1);  // 1ms delay
    }
}


__attribute__((noreturn)) void StartTRANSTASK(void *argument)
{
    SerialTxTaskInit();

    for (;;)
    {
        SerialTxTask();
        osDelay(10);  // 1ms delay
    }
}

__attribute__((noreturn)) void StartMOTORTASK(void *argument)
{
    motor_task_init();

    for (;;)
    {
        motor_task();
        osDelay(1);  // 1ms delay
    }
}


