#pragma once

#include "cmsis_os.h"
// #include <string.h>

// /* 接收中断字节 */



// /* 状态机相关变量 */
// typedef enum {
//     RX_WAIT_HEADER,
//     RX_WAIT_LEN,
//     RX_WAIT_PAYLOAD
// } RxParseState;

// #define RX_TARGET_HEADER  0xA6
// #define RX_EXPECTED_LEN   8  // 两个 float
// static RxParseState rx_state = RX_WAIT_HEADER;
// static uint8_t rx_payload[RX_EXPECTED_LEN];
// static uint8_t rx_len = 0;
// static uint8_t rx_index = 0;

// /* 存储解析结果 */
// float cmd_linear_x = 0.0f;
// float cmd_angular_z = 0.0f;


#define UART_FRAME_MAX_LEN 64

typedef struct {
    uint8_t data[UART_FRAME_MAX_LEN];
    uint8_t length;
} UARTMessage;




void SerialTxTask(void);
void SerialTxTaskInit(void);
osMessageQueueId_t Getimu_UartQueueHandle(void);
osMessageQueueId_t Getodom_UartQueueHandle(void);