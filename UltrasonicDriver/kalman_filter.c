#include "kalman_filter.h"

void kalman_init(KalmanFilter *kf, float init_value, float init_P, float Q, float R) {
    kf->x = init_value;
    kf->P = init_P;
    kf->Q = Q;
    kf->R = R;
}

float kalman_update(KalmanFilter *kf, float z) {
    // Predict
    kf->P = kf->P + kf->Q;

    // Kalman Gain
    float K = kf->P / (kf->P + kf->R);

    // Update estimate with measurement z
    kf->x = kf->x + K * (z - kf->x);

    // Update error covariance
    kf->P = (1 - K) * kf->P;

    return kf->x;
}
