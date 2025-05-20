#include "ins_task.h"
#include "NJY901S.h"
#include "gpio.h"
#include "usart.h"
#include "string.h"
#include "transit_task.h"
//---------------------------
#define START_BYTE 0xA5
#define TYPE_IMU   0x01
#define IMU_DATA_LEN 40  // 10 floats * 4 bytes

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
    osMessageQueuePut(GetUartQueueHandle(), &msg, 0, 0);
}
void data_transmit(){
    _imu_data._stcTime = stcTime;
    _imu_data._stcAcc = stcAcc;
    _imu_data._stcGyro = stcGyro;
    _imu_data._stcAngle = stcAngle;
    _imu_data._stcMag = stcMag;
    _imu_data._stcDStatus = stcDStatus;
    _imu_data._stcPress = stcPress;
    _imu_data._stcGPSV = stcGPSV;
    _imu_data._stcLonLat = stcLonLat;
    imu.qx = 0.1f;
    imu.qy = 0.0f;
    imu.qz = 0.0f;
    imu.qw = 1.0f;
    imu.gx = _imu_data._stcGyro.w[0];
    imu.gy = _imu_data._stcGyro.w[1];
    imu.gz = _imu_data._stcGyro.w[2];
    imu.ax = _imu_data._stcAcc.a[0];
    imu.ay = _imu_data._stcAcc.a[1];
    imu.az = _imu_data._stcAcc.a[2];
}

void ins_init(void){
    clear_imu_data();
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


    }

}
