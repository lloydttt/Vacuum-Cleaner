#include "transit_task.h"
#include "usart.h"



static osMessageQueueId_t uartTxQueueHandle;

osMessageQueueId_t GetUartQueueHandle(void) {
    return uartTxQueueHandle;
}

void SerialTxTask(void) {
    UARTMessage msg;
    for (;;) {
        if (osMessageQueueGet(uartTxQueueHandle, &msg, NULL, osWaitForever) == osOK) {
            HAL_UART_Transmit(&huart3, msg.data, msg.length, HAL_MAX_DELAY);
        }
    }
}

void SerialTxTaskInit(void) {
    uartTxQueueHandle = osMessageQueueNew(20, sizeof(UARTMessage), NULL);
}





