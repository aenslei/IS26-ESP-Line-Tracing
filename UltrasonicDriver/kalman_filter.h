#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

typedef struct {
    float x;    // estimated distance
    float P;    // estimate uncertainty
    float Q;    // process noise covariance
    float R;    // measurement noise covariance
} KalmanFilter;

void kalman_init(KalmanFilter *kf, float init_value, float init_P, float Q, float R);
float kalman_update(KalmanFilter *kf, float measurement);

#endif
