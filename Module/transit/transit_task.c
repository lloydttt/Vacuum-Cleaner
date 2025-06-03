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

void SerialTxTaskInit(void) {
    imuQueue  = osMessageQueueNew(20, sizeof(UARTMessage), NULL);
    odomQueue = osMessageQueueNew(20, sizeof(UARTMessage), NULL);
}





