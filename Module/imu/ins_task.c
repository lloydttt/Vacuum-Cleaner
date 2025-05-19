#include "ins_task.h"
#include "gpio.h"
#include "usart.h"

struct imu_data _imu_data = {0};
uint8_t rx_byte;
extern int ldoggy;
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
    
}

void ins_init(void){
    clear_imu_data();
    HAL_GPIO_WritePin(USER_LED_GPIO_Port, USER_LED_Pin, GPIO_PIN_SET);
    HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
}
// void ins_task(void){
//     HAL_UART_Receive(&huart1, &rx_byte, 1, 100); // 阻塞100ms接收一个字节
//     CopeSerialData(rx_byte);
//     // data_transmit();
//     ldoggy += 1;
// }   

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
