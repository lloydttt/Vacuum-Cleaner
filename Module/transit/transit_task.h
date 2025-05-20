#pragma once

#include "cmsis_os.h"

#define UART_FRAME_MAX_LEN 64

typedef struct {
    uint8_t data[UART_FRAME_MAX_LEN];
    uint8_t length;
} UARTMessage;

void SerialTxTask(void);
void SerialTxTaskInit(void);
osMessageQueueId_t GetUartQueueHandle(void);