#include "robot_task.h"
#include "ins_task.h"
#include "motor_task.h"
#include "transit_task.h"

osThreadId_t insTaskHandle;
osThreadId_t motorTaskHandle;
osThreadId_t transTaskHandle;
/**
 * @brief 初始化操作系统任务
 * @details 此函数用于初始化多个操作系统任务，包括传感器任务、运动任务和状态转换任务。
 *          每个任务都通过 osThreadNew 函数创建，并设置了相应的任务属性，如名称、优先级和堆栈大小。
 * @note 
 * - insTask: 优先级为 osPriorityAboveNormal，堆栈大小为 1024。
 * - motorTask: 优先级为 osPriorityNormal，堆栈大小为 1024。
 * - transTask: 优先级为 osPriorityNormal，堆栈大小为 1024。
 * 
 * @attention 确保在调用此函数之前，操作系统内核已经启动，否则任务创建可能失败。
 */
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

/**
 * @brief 启动INS任务的函数
 * @details 此函数初始化INS模块，并在一个无限循环中持续调用INS任务函数。
 * @note 该函数通过调用 `ins_init` 完成初始化，然后在循环中调用 `ins_task` 执行任务。
 *       每次循环结束后，通过 `osDelay(1)` 实现1毫秒的延迟。
 * @param argument 传递给任务的参数，未在此函数中使用。
 * @warning 此函数被标记为 `__attribute__((noreturn))`，意味着它不会返回。
 */
__attribute__((noreturn)) void StartINSTASK(void *argument)
{
    ins_init();

    for (;;)
    {
        ins_task();
        osDelay(1);  // 1ms delay
    }
}


/**
 * @brief 启动传输任务函数
 * @details 此函数初始化串口传输任务，并进入一个无限循环，
 *          在循环中调用串口传输任务函数并延时10毫秒。
 * @note 此函数被标记为 __attribute__((noreturn))，表示不会返回。
 *       在循环中，SerialTxTask 函数用于处理串口传输任务，
 *       osDelay(10) 用于实现10毫秒的延时。
 * @param argument 传递给任务的参数，未在此函数中使用。
 */
__attribute__((noreturn)) void StartTRANSTASK(void *argument)
{
    SerialTxTaskInit();

    for (;;)
    {
        SerialTxTask();
        osDelay(10);  // 10ms delay
    }
}

/**
 * @brief 启动电机任务函数
 * @details 此函数初始化电机任务并进入无限循环，周期性地调用电机任务函数。
 * @note 在循环中，每次调用 motor_task 函数后会延迟 1 毫秒，以确保任务的周期性执行。
 * @param argument 传递给任务的参数，未在此函数中使用。
 * @remark 此函数使用了 __attribute__((noreturn)) 属性，表示函数不会返回。
 */
__attribute__((noreturn)) void StartMOTORTASK(void *argument)
{
    motor_task_init();

    for (;;)
    {
        motor_task();
        osDelay(1);  // 1ms delay
    }
}


