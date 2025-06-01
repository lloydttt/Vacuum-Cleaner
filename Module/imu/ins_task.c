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
extern uint8_t rx3_byte;    // USART1 接收缓冲
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
    .qx = 0.1f, .qy = 0.0f, .qz = 0.0f, .qw = 1.0f,
    .gx = 0.01f, .gy = -0.02f, .gz = 0.005f,
    .ax = 0.0f, .ay = 9.81f, .az = 0.0f
};
//---------------------------


struct imu_data _imu_data = {0};
uint8_t rx_byte;
extern int ldoggy;
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
void sendIMUData(UART_HandleTypeDef *huart, IMUData *imu) {
    // uint8_t tx_buf[1 + 1 + 1 + IMU_DATA_LEN + 1];  // header + type + len + data + checksum
    // uint8_t *p = tx_buf;
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

    // Send via UART
    // HAL_UART_Transmit(huart, tx_buf, sizeof(tx_buf), HAL_MAX_DELAY);
    msg.length = 44;
    osMessageQueuePut(Getimu_UartQueueHandle(), &msg, 0, 0);
}
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

void ins_init(void){
    clear_imu_data();
    IMU_Kalman_Init(); 
    HAL_GPIO_WritePin(USER_LED_GPIO_Port, USER_LED_Pin, GPIO_PIN_SET);
    HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
}
void ins_task(void){

        if(flag == 1){
            data_transmit();
            sendIMUData(&huart3, &imu);
        }
}   

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

    if (huart == &huart1)
    {

        // WitSerialDataIn(rx_byte);
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
