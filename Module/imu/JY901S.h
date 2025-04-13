#ifndef __JY901S_H__
#define __JY901S_H__

#include <string.h>
#include <stdio.h>
#include "wit_c_sdk.h"
#include "usart.h"


#define ACC_UPDATE		0x01
#define GYRO_UPDATE		0x02
#define ANGLE_UPDATE	0x04
#define MAG_UPDATE		0x08
#define READ_UPDATE		0x80

static volatile char s_cDataUpdate = 0, s_cCmd = 0xff;
float fAcc[3], fGyro[3], fAngle[3];

typedef struct
{
    float FACC[3];	
    float FGYRO[3];	
    float FANGLE[3];	
}IMU_DATA;

IMU_DATA _IMUData;


// static void CmdProcess(void);
// static void AutoScanSensor(void);     115200 Original baud rate
void Uart1Send(uint8_t *p_data, unsigned int uiSize);
static void SensorUartSend(uint8_t *p_data, uint32_t uiSize);
static void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum);
static void Delayms(uint16_t ucMs);
void JY901S_Init(void);
IMU_DATA JY901S_GetData(void); 

#endif // __JY901S_H__