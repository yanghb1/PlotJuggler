#include "car_model.h"

// 计算转弯半径
double calculateTurningRadius(double wheelbase, double steeringAngleRadians) {
    return wheelbase / tan(steeringAngleRadians);
}

/**
 * @brief updateVehicleState 更新车辆状态
 * @param state
 * @param speed // 速度，单位：米/秒
 * @param steeringAngleRadians // 转向角，单位：弧度
 * @param wheelbase 轴距，单位：米
 * @param deltaT // 时间步长，单位：秒
 * @return
 */
VehicleState updateVehicleState(const VehicleState& state, double speed, double steeringAngleRadians, double wheelbase, double deltaT) {
    double R = calculateTurningRadius(wheelbase, steeringAngleRadians);

    double thetaDot = speed / R;

    VehicleState newState = state;
    newState.x = state.x + speed * qCos(state.theta) * deltaT;
    newState.y = state.y + speed * qSin(state.theta) * deltaT;
    newState.theta = state.theta + thetaDot * deltaT;

    return newState;
}

/**
 * @brief updateVehicleState 更新车辆状态
 * @param state
 * @param speed // 速度，单位：米/秒
 * @param deltaT // 时间步长，单位：秒
 * @param epsStrgAng // 方向盘转角
 * @return
 */
VehicleState updateVehicleState(const VehicleState& state, double speed, double deltaT, double epsStrgAng) {
    if(false){
        double R = getR(epsStrgAng) / 2.0;

        double thetaDot = speed / R;

        VehicleState newState = state;
        newState.x = state.x + speed * deltaT * qSin(state.theta);
        newState.y = state.y + speed * deltaT * qCos(state.theta);

        newState.theta = state.theta + (epsStrgAng > 0 ? thetaDot * deltaT : - thetaDot * deltaT);

        return newState;
    }

    if(true){
        //轴距 L9：3.105 L6：2.92
        double R = 2.92 / qTan(qDegreesToRadians(getSteer(epsStrgAng)));

        double thetaDot = speed / R;  //角速度

        VehicleState newState = state;
        newState.x = state.x + speed * deltaT * qSin(state.theta); //deltaT时长
        newState.y = state.y + speed * deltaT * qCos(state.theta);

        newState.theta = state.theta + thetaDot * deltaT;

        return newState;
    }

}

/**
 * @brief getR 获取转弯半径
 * @param x 方向盘转角
 * @return
 */
double getR(double x){
    double predicted_y = std::exp(8.728698696056606) * std::pow(std::abs(x),-0.992918644675939);
    return predicted_y;
}

/**
 * @brief getSteer 获取前轮转角（左右平均）
 * @param x 方向盘转角
 * @return
 */
double getSteer(double x){
    static bool init = false;
    static QList<QPair<double, double>> data;
    if(!init){
        data = HtmlTableParser::parse("/mnt/sda2/code/qt/opengl_demo/config/steer.htm");
        init = true;
    }

    // 处理 x 小于最小键的情况
    if (x <= data.first().first) {
        return data.first().second;
    }

    // 处理 x 大于最大键的情况
    if (x >= data.last().first) {
        return data.last().second;
    }

    // 查找 x 所在的区间
    for (int i = 0; i < data.size() - 1; ++i) {
        double x0 = data.at(i).first;
        double y0 = data.at(i).second;
        double x1 = data.at(i + 1).first;
        double y1 = data.at(i + 1).second;

        if (x >= x0 && x <= x1) {
            // 线性插值计算
            double y = y0 + (x - x0) * (y1 - y0) / (x1 - x0);
            return y;
        }
    }
}
