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


extern double calculateTurningRadius(double wheelbase, double steeringAngleRadians);
extern VehicleState updateVehicleState(const VehicleState& state, double speed, double steeringAngleRadians, double wheelbase, double deltaT);
extern VehicleState updateVehicleState(const VehicleState& state, double speed, double deltaT, double epsStrgAng);
extern double getR(double x);
extern double getSteer(double x);




#endif // CAR_MODEL_H
