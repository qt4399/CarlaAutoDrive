//这里所有预测小车行进预测的速度固定为5m/s
#include "../include/car.h"
#include "../include/visualizer.h"
#include "cstdio"
#include <cmath>
void Car::set_global_x(int index, float value) {
        if (index >= 0 && index < 1000) {
            global_x[index] = value;  // 直接修改数组元素
        } else {
            throw std::out_of_range("global_x 索引越界！必须在 0-999 之间");
        }
    }
void Car::set_global_y(int index, float value) {
        if (index >= 0 && index < 1000) {
            global_y[index] = value;  // 直接修改数组元素
        } else {
            throw std::out_of_range("global_y 索引越界！必须在 0-999 之间");
        }
    }
float Car::AngleSpeed(float wish_steer,Car& car){
    float angle_speed;
    angle_speed=(5*tan(wish_steer*(-0.610865)))/FBWheelDis;
    return angle_speed;
}
void Car::update_steer(Car& car){
    car.steer=car.wish_steer[0];
}
