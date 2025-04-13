#include "JY901S.h"

void Uart1Send(uint8_t *p_data, unsigned int uiSize){
	unsigned int i;
	for(i = 0; i < uiSize; i++)
	{
		while(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TXE) == RESET);
		HAL_UART_Transmit(&huart1, p_data++, 1, HAL_MAX_DELAY);		
	}
	while(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TXE) == RESET);
}
static void SensorUartSend(uint8_t *p_data, uint32_t uiSize){

    Uart1Send(p_data, uiSize);

}


static void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum){

	int i;
    for(i = 0; i < uiRegNum; i++)
    {
        switch(uiReg)
        {
//            case AX:
//            case AY:
            case AZ:
				s_cDataUpdate |= ACC_UPDATE;
            break;
//            case GX:
//            case GY:
            case GZ:
				s_cDataUpdate |= GYRO_UPDATE;
            break;
//            case HX:
//            case HY:
            case HZ:
				s_cDataUpdate |= MAG_UPDATE;
            break;
//            case Roll:
//            case Pitch:
            case Yaw:
				s_cDataUpdate |= ANGLE_UPDATE;
            break;
            default:
				s_cDataUpdate |= READ_UPDATE;
			break;
        }
		uiReg++;
    }

}

static void Delayms(uint16_t ucMs){

	HAL_Delay(ucMs);

}




void JY901S_Init(void){
    WitInit(WIT_PROTOCOL_NORMAL, 0x50);
	WitSerialWriteRegister(SensorUartSend);
	WitRegisterCallBack(SensorDataUpdata);
	WitDelayMsRegister(Delayms);
    // 协议注册相关函数、

}

IMU_DATA JY901S_GetData(void){
	int i;
    if(s_cDataUpdate)
    {
        for(i = 0; i < 3; i++)
           {
                fAcc[i] = sReg[AX+i] / 32768.0f * 16.0f;
                fGyro[i] = sReg[GX+i] / 32768.0f * 2000.0f;
                fAngle[i] = sReg[Roll+i] / 32768.0f * 180.0f;
                _IMUData.FACC[i] = fAcc[i];
                _IMUData.FGYRO[i] = fGyro[i];
                _IMUData.FANGLE[i] = fAngle[i];
           }
           if(s_cDataUpdate & ACC_UPDATE)
           {
                // printf("acc:%.3f %.3f %.3f\r\n", fAcc[0], fAcc[1], fAcc[2]);
                s_cDataUpdate &= ~ACC_UPDATE;
           }
           if(s_cDataUpdate & GYRO_UPDATE)
           {
            //    printf("gyro:%.3f %.3f %.3f\r\n", fGyro[0], fGyro[1], fGyro[2]);
               s_cDataUpdate &= ~GYRO_UPDATE;
           }
           if(s_cDataUpdate & ANGLE_UPDATE)
           {
            //    printf("angle:%.3f %.3f %.3f\r\n", fAngle[0], fAngle[1], fAngle[2]);
               s_cDataUpdate &= ~ANGLE_UPDATE;
           }
           if(s_cDataUpdate & MAG_UPDATE)
           {
            //    printf("mag:%d %d %d\r\n", sReg[HX], sReg[HY], sReg[HZ]);
               s_cDataUpdate &= ~MAG_UPDATE;
           }
    } 
    return _IMUData;
}




