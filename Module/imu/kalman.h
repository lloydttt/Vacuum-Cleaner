#pragma once

typedef struct {
    float angle;       // 估计的角度
    float bias;        // 陀螺仪偏差
    float rate;        // 未滤波的角速度

    float P[2][2];     // 误差协方差矩阵

    float Q_angle;     // 过程噪声协方差（角度）
    float Q_bias;      // 过程噪声协方差（偏差）
    float R_measure;   // 测量噪声协方差
} Kalman_t;

Kalman_t kalmanAcc[3];   // AccX, AccY, AccZ
Kalman_t kalmanGyro[3];  // GyroX, GyroY, GyroZ
Kalman_t kalmanAngle[3]; // Roll, Pitch, Yaw


void Kalman_Init(Kalman_t *kalman);
float Kalman_GetAngle(Kalman_t *kalman, float newAngle, float newRate, float dt);
void Kalman_All_Init();
void process_imu_kalman_with_dt();
