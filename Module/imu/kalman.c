// #include "kalman.h"


// uint32_t last_tick = 0;
// void Kalman_Init(Kalman_t *kalman) {
//     kalman->angle = 0.0f;
//     kalman->bias = 0.0f;
//     kalman->P[0][0] = 0.0f;
//     kalman->P[0][1] = 0.0f;
//     kalman->P[1][0] = 0.0f;
//     kalman->P[1][1] = 0.0f;

//     kalman->Q_angle = 0.001f;
//     kalman->Q_bias = 0.003f;
//     kalman->R_measure = 0.03f;
// }


// float Kalman_GetAngle(Kalman_t *kalman, float newAngle, float newRate, float dt)
// {
//     // 预测
//     kalman->rate = newRate - kalman->bias;
//     kalman->angle += dt * kalman->rate;

//     kalman->P[0][0] += dt * (dt*kalman->P[1][1] - kalman->P[0][1] - kalman->P[1][0] + kalman->Q_angle);
//     kalman->P[0][1] -= dt * kalman->P[1][1];
//     kalman->P[1][0] -= dt * kalman->P[1][1];
//     kalman->P[1][1] += kalman->Q_bias * dt;

//     // 更新
//     float S = kalman->P[0][0] + kalman->R_measure;
//     float K[2];
//     K[0] = kalman->P[0][0] / S;
//     K[1] = kalman->P[1][0] / S;

//     float y = newAngle - kalman->angle;

//     kalman->angle += K[0] * y;
//     kalman->bias += K[1] * y;

//     float P00_temp = kalman->P[0][0];
//     float P01_temp = kalman->P[0][1];

//     kalman->P[0][0] -= K[0] * P00_temp;
//     kalman->P[0][1] -= K[0] * P01_temp;
//     kalman->P[1][0] -= K[1] * P00_temp;
//     kalman->P[1][1] -= K[1] * P01_temp;

//     return kalman->angle;
// }

// void Kalman_All_Init() {
//     for(int i = 0; i < 3; i++) {
//         Kalman_Init(&kalmanAcc[i]);
//         Kalman_Init(&kalmanGyro[i]);
//         Kalman_Init(&kalmanAngle[i]);
//     }
// }

// void process_imu_kalman_with_dt()
// {
//     uint32_t now = HAL_GetTick();        // 当前时间（单位：ms）
//     float dt = (now - last_tick) / 1000.0f; // 计算时间差，单位转换为秒
//     if (dt <= 0 || dt > 1.0f) dt = 0.01f;  // 避免意外值

//     last_tick = now;

//     for (int i = 0; i < 3; i++) {
//         float acc = stcAcc.a[i] / 32768.0f * 16.0f;
//         float gyro = stcGyro.w[i] / 32768.0f * 2000.0f;
//         float angle = stcAngle.Angle[i] / 32768.0f * 180.0f;

//         _imu_data.filtered_acc[i]   = Kalman_GetAngle(&kalmanAcc[i], acc, 0, dt);
//         _imu_data.filtered_gyro[i]  = Kalman_GetAngle(&kalmanGyro[i], gyro, 0, dt);
//         _imu_data.filtered_angle[i] = Kalman_GetAngle(&kalmanAngle[i], angle, gyro, dt);
//     }
// }



// //@todo: 优化滤波效果，包结束标志，通信位置思考测试