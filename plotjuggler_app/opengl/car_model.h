#ifndef CAR_MODEL_H
#define CAR_MODEL_H

#include <iostream>
#include <QtMath>
#include <vector>
#include <Eigen/Dense>
#include "htmltableparser.h"

struct VehicleState {
    double x;     // 车辆位置 x
    double y;     // 车辆位置 y
    double theta; // 车辆朝向角（弧度）
};

enum PointSide {
    LEFT = 0,
    RTGHT,
    ON_LINE
};


extern double calculateTurningRadius(double wheelbase, double steeringAngleRadians);
extern VehicleState updateVehicleState(const VehicleState& state, double speed, double steeringAngleRadians, double wheelbase, double deltaT);
extern VehicleState updateVehicleState(const VehicleState& state, double speed, double deltaT, double epsStrgAng);
extern double getR(double x);
extern double getSteer(double x);

// 判断点 C 在 A/B 连线的哪一侧
extern PointSide cal_point_side(const VehicleState& A, const VehicleState& B, const VehicleState& C);
extern double cal_point_distance(const VehicleState& A, const VehicleState& B);
// 判断同一条直线上的AB AC 连线方向是否相同
extern bool is_same_direction(const VehicleState& A, const VehicleState& B, const VehicleState& C);
// 三角形余弦定理计算a边对应的角度
extern double cal_point_angle(double a, double b, double c);

#endif // CAR_MODEL_H
