#ifndef KALMANFILTER_H
#define KALMANFILTER_H


class KalmanFilter
{
public:
    KalmanFilter(double processNoiseCov, double measurementNoiseCov, double estimationErrorCov);
    double update(double measurement);

private:
    double Q;  // 过程噪声协方差
    double R;  // 测量噪声协方差
    double P;  // 估计误差协方差
    double K;  // 卡尔曼增益
    double x;  // 估计值
};

#endif // KALMANFILTER_H
