//这里所有预测小车行进预测的速度固定为5m/s
#include "../include/visualizer.h"
void Car::SteerToBaseGoal(int step,float step_time,Car& car){
    float now_angle=car.angle*0.0174533;
    float now_point_x=car.pose_right_x;
    float now_point_y=car.pose_right_y;
    for (int i=0;i<step;i++){
            float min_dis=99;
            float aim_angle=0;
            float dis_steer_change=0;
            for(int k=0;k<step;k++){
                float dis=std::pow((std::pow(now_point_y-car.global_y[k], 2.0f)+std::pow(now_point_x-car.global_x[k], 2.0f)),0.5f);
                if (dis<min_dis){
                    aim_angle=std::atan((car.global_y[k+1]-car.global_y[k])/(car.global_x[k+1]-car.global_x[k]+0.0000001));
                    if (car.global_y[k+1]-car.global_y[k]<0&&aim_angle>0){aim_angle=-3.1415926536+aim_angle;}
                    else if(car.global_y[k+1]-car.global_y[k]>0&&aim_angle<0){aim_angle=3.1415926536+aim_angle;}
                    if((now_point_y-car.global_y[k])>=0&&(now_point_x-car.global_x[k])>=0){dis_steer_change=dis;}
                    else if((now_point_y-car.global_y[k])<0&&(now_point_x-car.global_x[k])<0){dis_steer_change=dis;}
                    else if((now_point_y-car.global_y[k])<0&&(now_point_x-car.global_x[k])>=0){dis_steer_change=-dis;}
                    else if((now_point_y-car.global_y[k])>=0&&(now_point_x-car.global_x[k])<0){dis_steer_change=-dis;}
                    min_dis=dis;
                }
            }
            float plan_angle_change=now_angle-aim_angle;
            if (plan_angle_change>=3.1415926536){plan_angle_change=-6.2831853072+plan_angle_change;}
            else if (plan_angle_change<-3.1415926536){plan_angle_change=6.2831853072+plan_angle_change;}
            car.wish_steer[i]=2*(plan_angle_change)+dis_steer_change*0.01;//计算目标角度4
            //printf("%f,%f",now_angle,aim_angle);
            if (car.wish_steer[i]>=1){car.wish_steer[i]=1;}else if(car.wish_steer[i]<=-1){car.wish_steer[i]=-1;}
            float angle_speed=AngleSpeed(car.wish_steer[i],car);
            float angle_change=angle_speed*step_time;
            float dis_change=5*step_time;
            now_point_x+=dis_change*cos(now_angle);
            now_point_y+=dis_change*sin(now_angle);
            now_angle+=angle_change;
            car.predict_x[i]=now_point_x;
            car.predict_y[i]=now_point_y;
            //printf("(%f,%f),%f,%d\n",car.predict_x[i],car.predict_y[i],car.wish_steer[i],i);
    }
}
//void Car::SteerToBaseGoal(float* wish_steer,int step,float step_time,Car& car){
//    float now_angle=car.angle*0.0174533;
//    float now_point_x=car.pose_right_x;
//    float now_point_y=car.pose_right_y;
//    for (int i=0;i<step;i++){
//        float angle_speed=AngleSpeed(wish_steer[int(i/5)],car);
//        float angle_change=angle_speed*step_time;
//        float dis_change=5*step_time;
//        now_point_x+=dis_change*cos(now_angle);
//        now_point_y+=dis_change*sin(now_angle);
//        now_angle+=angle_change;
//        car.predict_x[i]=now_point_x;
//        car.predict_y[i]=now_point_y;
//        }
//    }
//}
//车辆控制的主函数############################################
void Car::PlanAndControl(Car& car){
    if(global_path_num<15){
        float car_power=((((float)global_path_num-5))/20-car.speed*(1/(float)global_path_num)*0.1);
        if (car_power>=1){car_power=1;}
        else if(car_power<=-1){car_power=-1;}
        car.power=car_power;
        car.step=global_path_num;
    }
    else{
        float car_power=0.5;
        car.step=15;
        car.power=car_power;
    }
    SteerToBaseGoal(car.step,car.step_time,car);
    update_steer(car);
//    int best_a=0;
//    int best_b=0;
//    int best_c=0;
//    float best_score=-999;
//    for (int a=0;a<=10;a++){
//        for(int b=0;b<=10;b++){
//            for (int c=0;c<=10;c++){
//                    float score;
//                    car.wish_steer[0]=(float(a-5)/5);
//                    car.wish_steer[1]=(float(b-5)/5);
//                    car.wish_steer[2]=(float(c-5)/5);
//                    score=JudgeBestLine(car.wish_steer,car.judge_step,car.judge_step_time,car);
//                    if (score>=best_score){
//                        best_score=score;
//                        best_a=a;
//                        best_b=b;
//                        best_c=c;
//                    }
//            }
//        }
//    }
    //printf("%d,%d,%d,%f\n",best_a,best_b,best_c,best_score);
//    if (car.start_step<car.step){
//        car.wish_steer[car.start_step]=0;
//        car.start_step+=1;
//    }
//    else{
//        for(int i=0;i<car.step-1;i++){
//            car.wish_steer[i]=car.wish_steer[i+1];
//            if(i==9){car.wish_steer[i]=0;}
//        }
//    }map.cpp

}
//车辆控制的主函数############################################