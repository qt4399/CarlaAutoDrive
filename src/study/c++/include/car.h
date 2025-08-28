#ifndef CAR_H
#define CAR_H
class Car{
private:
    float High=1.44;
    float Long=4.69;
    float FrontWheelDis=1.58;
    float BehindWheelDis=1.58;
    float FBWheelDis=2.89;

public:
    int global_path_num=0;
    int step=15;
    bool AutoDriveOn=false;
    float step_time=0.4;
    float speed=0;
    float power=0;
    float steer=0;
    float pose_right_x=0;
    float pose_right_y=0;
    float angle=0;
    float predict_x[15]={};
    float predict_y[15]={};
    float wish_steer[15]={};
    float wish_speed[15]={};
    float global_x[1000]={};
    float global_y[1000]={};
    void set_global_x(int index, float value);
    void set_global_y(int index, float value);
    void update_steer(Car& car);
    float AngleSpeed(float wish_steer,Car& car);
    void SteerToBaseGoal(int step,float step_time,Car& car);//将-1到1的转向值(默认tesla,-35到35度)转化为局部规划器的转向带
    void PlanAndControl(Car& car);

};
#endif