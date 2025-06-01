#ifndef __INS_TASK_H__
#define __INS_TASK_H__
#include "NJY901S.h"

struct imu_data
{
    struct STime		_stcTime;
    struct SAcc 		_stcAcc;
    struct SGyro 		_stcGyro;
    struct SAngle 		_stcAngle;
    struct SMag 		_stcMag;
    struct SDStatus 	_stcDStatus;
    struct SPress 		_stcPress;
    struct SLonLat 		_stcLonLat;
    struct SGPSV 		_stcGPSV; 
};

float DegToRad(float degrees);
// todo change recieve mode to blocking mode, aims to control the frequency of IMU updating 
void data_transmit(void);

void ins_init(void);

void ins_task(void);

#endif