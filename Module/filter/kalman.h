#ifndef KALMAN_H
#define KALMAN_H

typedef struct {
    float x;      // 估计值
    float P;      // 协方差
    float Q;      // 过程噪声
    float R;      // 观测噪声
} KalmanFilter;

inline void Kalman_Init(KalmanFilter* kf, float q, float r, float init_value){
    kf->x = init_value;
    kf->P = 1.0f;
    kf->Q = q;
    kf->R = r;
}
inline float Kalman_Update(KalmanFilter* kf, float measurement) {
    // Prediction step
    float x_pred = kf->x;
    float P_pred = kf->P + kf->Q;

    // Update step
    float K = P_pred / (P_pred + kf->R);
    kf->x = x_pred + K * (measurement - x_pred);
    kf->P = (1 - K) * P_pred;

    return kf->x;
}

#endif // KALMAN_H