#include "ins_task.h"
#include "NJY901S.h"
#include "gpio.h"
#include "usart.h"
#include "string.h"
#include "transit_task.h"
#include "math.h"
#include "control.h"
#include "kalman.h"

extern float d[9];
extern uint8_t rx3_byte;    // USART3 接收缓冲
extern float cmd_linear_x;
extern float cmd_angular_z;

extern uint8_t rx_state;
extern uint8_t rx_len;
extern uint8_t rx_index;
extern uint8_t rx_payload[8];
KalmanFilter imu_filters[9];  // 对应 d[0]~d[8] 的 9 个滤波器
float filtered_d[9];          // 存储滤波后的值

//---------------------------
#define START_BYTE 0xA5
#define TYPE_IMU   0x01
#define IMU_DATA_LEN 40  // 10 floats * 4 bytes
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

typedef struct {
    float qx, qy, qz, qw;
    float gx, gy, gz;
    float ax, ay, az;
} IMUData;


IMUData imu = {
    .qx = 0.0f, .qy = 0.0f, .qz = 0.0f, .qw = 0.0f,
    .gx = 0.0f, .gy = 0.0f, .gz = 0.0f,
    .ax = 0.0f, .ay = 0.0f, .az = 0.0f
};
//---------------------------


struct imu_data _imu_data = {0};
uint8_t rx_byte;
extern int flag;
//---------------------------
float DegToRad(float degrees) {
    return degrees * (M_PI / 180.0f);
}
void IMU_Kalman_Init(void) {
    for (int i = 0; i < 9; i++) {
        Kalman_Init(&imu_filters[i], 0.03f, 0.5f, d[i]);  // 你可以根据实际噪声调整 Q、R
    }
}
/**
 * @brief 将欧拉角转换为四元数表示。
 * 
 * @details 此函数接收滚转、俯仰和航向角（以弧度为单位），并将其转换为四元数表示。
 *          转换遵循ZYX旋转顺序（航向 -> 俯仰 -> 滚转）。转换后的四元数分量存储在
 *          提供的IMUData结构中。
 * 
 * @param roll 滚转角（弧度）。
 * @param pitch 俯仰角（弧度）。
 * @param yaw 航向角（弧度）。
 * @param imu 指向IMUData结构的指针，用于存储结果四元数。
 */
void EulerToQuaternion(float roll, float pitch, float yaw, IMUData *imu) {
    // 欧拉角的一半
    float cr = cosf(roll  * 0.5f);
    float sr = sinf(roll  * 0.5f);
    float cp = cosf(pitch * 0.5f);
    float sp = sinf(pitch * 0.5f);
    float cy = cosf(yaw   * 0.5f);
    float sy = sinf(yaw   * 0.5f);

    // ZYX顺序（即航向->俯仰->滚转）
    imu->qw = cr * cp * cy + sr * sp * sy;
    imu->qx = sr * cp * cy - cr * sp * sy;
    imu->qy = cr * sp * cy + sr * cp * sy;
    imu->qz = cr * cp * sy - sr * sp * cy;


}
/**
 * @brief 发送IMU数据到UART
 * @details 此函数将IMU数据打包成UART消息并发送到指定的UART队列中。
 *          数据包包括起始字节、数据类型、数据长度、IMU数据负载以及校验和。
 * @param huart 指向UART句柄的指针，用于指定发送数据的UART接口。
 * @param imu 指向IMU数据结构的指针，包含需要发送的IMU数据。
 * @note 数据包的校验和通过对IMU数据负载的所有字节求和计算得到。
 *       数据包的总长度固定为44字节。
 */
void sendIMUData(UART_HandleTypeDef *huart, IMUData *imu) {
    UARTMessage msg;
    uint8_t *p = msg.data;
    // Fill header
    *p++ = START_BYTE;
    *p++ = TYPE_IMU;
    *p++ = IMU_DATA_LEN;

    // Copy data payload
    memcpy(p, imu, IMU_DATA_LEN);
    p += IMU_DATA_LEN;

    // Calculate checksum (simple sum of all bytes in data)
    uint8_t checksum = 0;
    for (int i = 0; i < IMU_DATA_LEN; i++) {
        checksum += *((uint8_t*)imu + i);
    }
    *p = checksum;
    msg.length = 44;
    osMessageQueuePut(Getimu_UartQueueHandle(), &msg, 0, 0);
}

/**
 * @brief 数据传输函数
 * @details 此函数对传感器数据进行处理，包括 Kalman 滤波和欧拉角到四元数的转换，
 *          并将处理后的数据存储到 imu 结构体中。
 * @note 
 * 1. 对 d[0] ~ d[8] 的数据进行 Kalman 滤波，滤波结果存储在 filtered_d 数组中。
 * 2. 使用 d[6]、d[7] 和 d[8] 的数据（经过角度到弧度的转换）计算四元数，并存储到 imu 中。
 * 3. 将 d[3] ~ d[5] 的数据（经过角度到弧度的转换）赋值给 imu 的 gx、gy 和 gz。
 * 4. 将 d[0] ~ d[2] 的数据直接赋值给 imu 的 ax、ay 和 az。
 */
void data_transmit(){
    // 对 d[0] ~ d[8] 进行 Kalman 滤波
    for (int i = 0; i < 9; i++) {
        filtered_d[i] = Kalman_Update(&imu_filters[i], d[i]);
    }
    EulerToQuaternion(DegToRad(d[6]), DegToRad(d[7]), DegToRad(d[8]), &imu);
    imu.gx = DegToRad(d[3]);
    imu.gy = DegToRad(d[4]);
    imu.gz = DegToRad(d[5]);
    imu.ax = d[0];
    imu.ay = d[1];
    imu.az = d[2];
}

/**
 * @brief 初始化惯性导航系统 (INS)
 * @details 此函数用于初始化惯性导航系统，包括清除IMU数据、初始化卡尔曼滤波器、
 *          设置用户LED状态以及启动UART接收中断。
 * @note 调用此函数时：
 *       1. clear_imu_data() 用于清除IMU相关数据。
 *       2. IMU_Kalman_Init() 用于初始化卡尔曼滤波器。
 *       3. HAL_GPIO_WritePin() 设置用户LED为点亮状态。
 *       4. HAL_UART_Receive_IT() 启动UART接收中断，接收一个字节的数据。
 */
void ins_init(void){
    clear_imu_data();
    IMU_Kalman_Init(); 
    HAL_GPIO_WritePin(USER_LED_GPIO_Port, USER_LED_Pin, GPIO_PIN_SET);
    HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
}
/**
 * @brief INS任务处理函数
 * @details 此函数用于处理惯性导航系统（INS）的任务逻辑。当标志位 `flag` 为 1 时，
 *          函数会调用数据传输函数 `data_transmit` 和 IMU 数据发送函数 `sendIMUData`。
 * @param 无
 * @return 无
 * @note 此函数依赖外部变量 `flag` 的值来决定是否执行数据传输操作。
 *       需要确保 `huart3` 和 `imu` 已正确初始化并传递给 `sendIMUData` 函数。
 */
void ins_task(void){

        if(flag == 1){
            data_transmit();
            sendIMUData(&huart3, &imu);
        }
}   

/**
 * @brief UART 接收完成回调函数
 * @details 此函数是 UART 接收完成中断的回调函数，根据接收的 UART 句柄执行不同的处理逻辑。
 *          如果接收到的是 huart1 的数据，则点亮用户 LED 并调用 CopeSerialData 函数处理数据，
 *          然后继续接收下一个字节。
 *          如果接收到的是 huart3 的数据，则根据状态机解析接收到的数据帧，
 *          并在接收完成后解析出线速度和角速度命令。
 * @param huart UART 句柄指针，用于区分是哪个 UART 触发的回调。
 * @note 此函数中使用了 HAL_UART_Receive_IT 函数来重新启动 UART 接收中断。
 *       对于 huart3 的数据解析，状态机包括三个状态：等待 header、等待长度和接收 payload。
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

    if (huart == &huart1)
    {

        HAL_GPIO_WritePin(USER_LED_GPIO_Port, USER_LED_Pin, GPIO_PIN_SET);
        CopeSerialData(rx_byte);
        // 继续接收下一个字节
        HAL_UART_Receive_IT(&huart1, &rx_byte, 1);  


    }else if (huart == &huart3)
    {
        uint8_t b = rx3_byte;

        switch (rx_state)
        {
        case 0:  // 等待 header
            if (b == 0xA6) rx_state = 1;
            break;
        case 1:  // 等待长度
            if (b == 8) {
                rx_len = 8;
                rx_index = 0;
                rx_state = 2;
            } else {
                rx_state = 0;
            }
            break;
        case 2:  // 接收 payload
            rx_payload[rx_index++] = b;
            if (rx_index >= rx_len) {
                memcpy(&cmd_linear_x,  &rx_payload[0], 4);
                memcpy(&cmd_angular_z, &rx_payload[4], 4);

                rx_state = 0;
            }
            break;
        }

        HAL_UART_Receive_IT(&huart3, &rx3_byte, 1);
    }

}
