#include "robot_task.h"
#include "ins_task.h"

osThreadId_t insTaskHandle;
osThreadId_t motorTaskHandle;

void OSTaskInit()
{
    const osThreadAttr_t insTaskAttr = {
        .name = "insTask",
        .priority = osPriorityAboveNormal,
        .stack_size = 1024
    };

    insTaskHandle = osThreadNew(StartINSTASK, NULL, &insTaskAttr);
    // motorTaskHandle = osThreadNew(StartMOTORTASK, NULL, &motorTaskAttr);
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






