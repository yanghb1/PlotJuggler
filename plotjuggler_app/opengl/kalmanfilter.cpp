#include "kalmanfilter.h"

KalmanFilter::KalmanFilter(double processNoiseCov, double measurementNoiseCov, double estimationErrorCov)
{
    // 初始化卡尔曼滤波器参数
    Q = processNoiseCov;  // 过程噪声协方差
    R = measurementNoiseCov;  // 测量噪声协方差
    P = estimationErrorCov;  // 估计误差协方差
    K = 0;  // 卡尔曼增益
    x = 0;  // 估计值
}

double KalmanFilter::update(double measurement)
{
    // 预测阶段
    // 更新卡尔曼增益
    K = P / (P + R);

    // 更新估计值
    x = x + K * (measurement - x);

    // 更新估计误差协方差
    P = (1 - K) * P + Q;

    return x;
}
