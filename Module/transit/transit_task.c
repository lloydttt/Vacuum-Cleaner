#include "transit_task.h"
#include "usart.h"


static osMessageQueueId_t imuQueue;
static osMessageQueueId_t odomQueue;
osMessageQueueId_t Getimu_UartQueueHandle(void) {
    return imuQueue;
}
osMessageQueueId_t Getodom_UartQueueHandle(void) {
    return odomQueue;
}


/**
 * @brief 串口发送任务
 * @details 该任务从两个消息队列 imuQueue 和 odomQueue 中获取消息，并通过串口发送。
 *          如果两个队列中均有消息，则依次发送 IMU 和里程计数据；
 *          如果仅有 IMU 数据，则只发送 IMU 数据。
 * @note 使用了 FreeRTOS 消息队列和 HAL 库的串口发送函数。
 */
void SerialTxTask(void) {
    UARTMessage imu_msg;
    UARTMessage odom_msg;

    for (;;) {

        if (osMessageQueueGet(imuQueue, &imu_msg, NULL, 0) == osOK && osMessageQueueGet(odomQueue, &odom_msg, NULL, 0) == osOK) {
            HAL_UART_Transmit(&huart3, imu_msg.data, imu_msg.length, HAL_MAX_DELAY);
            HAL_UART_Transmit(&huart3, odom_msg.data, odom_msg.length, HAL_MAX_DELAY);
        }else if(osMessageQueueGet(imuQueue, &imu_msg, NULL, 0) == osOK && osMessageQueueGet(odomQueue, &odom_msg, NULL, 0) != osOK) {
            HAL_UART_Transmit(&huart3, imu_msg.data, imu_msg.length, HAL_MAX_DELAY);
        }


    }
}


/**
 * @brief 串口发送任务初始化
 * @details 初始化两个消息队列 imuQueue 和 odomQueue，每个队列的容量为 20，消息大小为 UARTMessage 结构体的大小。
 */
void SerialTxTaskInit(void) {
    imuQueue  = osMessageQueueNew(20, sizeof(UARTMessage), NULL);
    odomQueue = osMessageQueueNew(20, sizeof(UARTMessage), NULL);
}





