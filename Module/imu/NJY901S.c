#include "NJY901S.h"
#include <string.h>
#include "usart.h"
#include "main.h"

struct STime		stcTime={0};
struct SAcc 		stcAcc={0};
struct SGyro 		stcGyro={0};
struct SAngle 		stcAngle={0};
struct SMag 		stcMag={0};
struct SDStatus 	stcDStatus={0};
struct SPress 		stcPress={0};
struct SLonLat 		stcLonLat={0};
struct SGPSV 		stcGPSV={0};

char txBuf[100];
float d[9];
signed  int x_a=0,y_a=0,z_a=0;
float temp1 = 208.98;
float temp2 = 16.384;
float temp3 = 182.04;
int flag = 0;
//	float g=9.8;
//	float temp1=32768/16/g;
//	float temp2=32768/2000.0;
//	float temp3=32768/180.0;


void CharToLong(char Dest[],char Source[])
{
	 *Dest 		= Source[3];
	 *(Dest+1) 	= Source[2];
	 *(Dest+2) 	= Source[1];
	 *(Dest+3) 	= Source[0];
}
/**
 * @brief 处理串口接收到的NJY901S传感器数据
 * @param ucData 接收到的单字节数据
 * @note 该函数用于解析NJY901S传感器通过串口发送的数据帧，根据数据帧的类型更新对应的传感器数据结构体。
 *       数据帧以0x55为起始字节，长度为11字节。根据第二字节的帧类型，解析不同的传感器数据：
 *       - 0x50: 时间数据
 *       - 0x51: 加速度数据
 *       - 0x52: 陀螺仪数据
 *       - 0x53: 角度数据
 *       - 0x54: 磁场数据
 *       - 0x55: 状态数据
 *       - 0x56: 气压和高度数据
 *       - 0x57: 经度和纬度数据
 *       - 0x58: GPS高度、航向和速度数据
 *       如果数据帧不完整或起始字节不正确，函数会丢弃当前数据并重新开始接收。
 */
void CopeSerialData(unsigned char ucData)
{   
	static unsigned char ucRxBuffer[12];
	static unsigned char ucRxCnt = 0;	
	
	ucRxBuffer[ucRxCnt++]=ucData;
    if (ucRxBuffer[0]!=0x55) //数据头不对，则重新开始寻找0x55数据头
	{
		// time_xxx++;
		ucRxCnt=0;
        // flag = 0;
		return;																																	  
	}else{
        if (ucRxCnt<11) {
            // flag = 0;
            return;
        }//数据不满11个，则返回
        else
        {
            switch(ucRxBuffer[1])
            {
                case 0x50: stcTime.ucYear 		= ucRxBuffer[2];
                            stcTime.ucMonth 	= ucRxBuffer[3];
                            stcTime.ucDay 		= ucRxBuffer[4];
                            stcTime.ucHour 		= ucRxBuffer[5];
                            stcTime.ucMinute 	= ucRxBuffer[6];
                            stcTime.ucSecond 	= ucRxBuffer[7];
                            stcTime.usMiliSecond=((unsigned short)ucRxBuffer[9]<<8)|ucRxBuffer[8];
                            break;
                case 0x51:	stcAcc.a[0] = ((unsigned short)ucRxBuffer[3]<<8)|ucRxBuffer[2];
                            stcAcc.a[1] = ((unsigned short)ucRxBuffer[5]<<8)|ucRxBuffer[4];
                            stcAcc.a[2] = ((unsigned short)ucRxBuffer[7]<<8)|ucRxBuffer[6];
                            d[0]=stcAcc.a[0]/temp1;			//单位：m/s^2
                            d[1]=stcAcc.a[1]/temp1;
                            d[2]=stcAcc.a[2]/temp1;
                            y_a+=d[1];
                            break;
                case 0x52:	stcGyro.w[0] = ((unsigned short)ucRxBuffer[3]<<8)|ucRxBuffer[2];
                            stcGyro.w[1] = ((unsigned short)ucRxBuffer[5]<<8)|ucRxBuffer[4];
                            stcGyro.w[2] = ((unsigned short)ucRxBuffer[7]<<8)|ucRxBuffer[6];
                            d[3]=stcGyro.w[0]/temp2;		//单位：°/s
                            d[4]=stcGyro.w[1]/temp2;
                            d[5]=stcGyro.w[2]/temp2;
                            break;
                case 0x53:	stcAngle.Angle[0] = ((short)ucRxBuffer[3]<<8)|ucRxBuffer[2];
                            stcAngle.Angle[1] = ((short)ucRxBuffer[5]<<8)|ucRxBuffer[4];
                            stcAngle.Angle[2] = ((short)ucRxBuffer[7]<<8)|ucRxBuffer[6];
                            stcAngle.T = ((short)ucRxBuffer[9]<<8)|ucRxBuffer[8];		
                            d[6]=stcAngle.Angle[0]/temp3;		//单位：°
                            d[7]=stcAngle.Angle[1]/temp3;
                            d[8]=stcAngle.Angle[2]/temp3;
                            break;
                case 0x54:	stcMag.h[0] = ((unsigned short)ucRxBuffer[3]<<8)|ucRxBuffer[2];
                            stcMag.h[1] = ((unsigned short)ucRxBuffer[5]<<8)|ucRxBuffer[4];
                            stcMag.h[2] = ((unsigned short)ucRxBuffer[7]<<8)|ucRxBuffer[6];
                            stcAngle.T = ((unsigned short)ucRxBuffer[9]<<8)|ucRxBuffer[8];
                            break;
                case 0x55:	stcDStatus.sDStatus[0] = ((unsigned short)ucRxBuffer[3]<<8)|ucRxBuffer[2];
                            stcDStatus.sDStatus[1] = ((unsigned short)ucRxBuffer[5]<<8)|ucRxBuffer[4];
                            stcDStatus.sDStatus[2] = ((unsigned short)ucRxBuffer[7]<<8)|ucRxBuffer[6];
                            stcDStatus.sDStatus[3] = ((unsigned short)ucRxBuffer[9]<<8)|ucRxBuffer[8];
                            break;
                case 0x56:	ucRxBuffer[2] = 0x12;ucRxBuffer[3] = 0x34;ucRxBuffer[4] = 0x56;ucRxBuffer[5] = 0x78;
                            CharToLong((char*)&stcPress.lPressure,(char*)&ucRxBuffer[2]);
                            CharToLong((char*)&stcPress.lAltitude,(char*)&ucRxBuffer[6]);
                            break;
                case 0x57:	CharToLong((char*)&stcLonLat.lLon,(char*)&ucRxBuffer[2]);
                            CharToLong((char*)&stcLonLat.lLat,(char*)&ucRxBuffer[6]);
                            break;
                case 0x58:	stcGPSV.sGPSHeight = ((unsigned short)ucRxBuffer[3]<<8)|ucRxBuffer[2];
                            stcGPSV.sGPSYaw = ((unsigned short)ucRxBuffer[5]<<8)|ucRxBuffer[4];
                            CharToLong((char*)&stcGPSV.lGPSVelocity,(char*)&ucRxBuffer[6]);
                            break;
            }

            ucRxCnt=0;
            flag = 1;
            
        }
    }

}

void clear(unsigned char *pta, int size )
{
    while(size>0)
    {
        *pta++ = 0;
        size --;
    }
}

void clear_imu_data(void){
	clear(( unsigned char *)&stcTime,sizeof(stcTime));
	clear(( unsigned char *)&stcAcc,sizeof(stcAcc));
	clear(( unsigned char *)&stcGyro,sizeof(stcGyro));
	clear(( unsigned char *)&stcAngle,sizeof(stcAngle));
	clear(( unsigned char *)&stcMag,sizeof(stcMag));
	clear(( unsigned char *)&stcDStatus,sizeof(stcDStatus));
	clear(( unsigned char *)&stcPress,sizeof(stcPress));
	clear(( unsigned char *)&stcLonLat,sizeof(stcLonLat));
	clear(( unsigned char *)&stcGPSV,sizeof(stcGPSV));
}

