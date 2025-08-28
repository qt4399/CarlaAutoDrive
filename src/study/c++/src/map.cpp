#include "../include/visualizer.h"
#include <string>       // 新增：支持 std::string
#include <functional>
#include <cmath>
#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>
#include <SDL2/SDL_image.h>
#include <chrono>
#include <thread>
void Visualizer::draw_map(int button2andy,bool show_situation){
    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
    SDL_Rect rect = {220, 0, 520, button2andy};  // x, y, 宽, 高
    SDL_RenderFillRect(renderer, &rect);
    if (show_situation==true&&button2andy==300){
        drawImage(220,0);
    }
}
void Visualizer::drawCar() {
    int car_w=6;
    int car_h=6;
    if (ego_car.pose_right_x<(-moveX*0.3+156+map_pos_start_x-5)&&ego_car.pose_right_x>-moveX*0.3+map_pos_start_x-5
        &&ego_car.pose_right_y>-(-moveY*0.3+90+map_pos_start_y-5)&&ego_car.pose_right_y<-(-moveY*0.3+map_pos_start_y-5)
    ){
        SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);
        int x=std::get<0>(from_map_point_to_windows_point(ego_car.pose_right_x,ego_car.pose_right_y));
        int y=std::get<1>(from_map_point_to_windows_point(ego_car.pose_right_x,ego_car.pose_right_y));
        SDL_Rect rect = {x-car_w/2,y-car_h/2,car_w, car_h};
        SDL_RenderFillRect(renderer, &rect);
    }
}
void Visualizer::drawAim(){
    if (goal_x<(-moveX*0.3+156+map_pos_start_x-5)&&goal_x>-moveX*0.3+map_pos_start_x-5
        &&goal_y>-(-moveY*0.3+90+map_pos_start_y-5)&&goal_y<-(-moveY*0.3+map_pos_start_y-5&&goal_x!=0&&goal_y!=0)
    ){
    int Aim_w=10;
    int Aim_h=10;
    SDL_SetRenderDrawColor(renderer, 0, 0, 255, 255);
    int x=std::get<0>(from_map_point_to_windows_point(goal_x,goal_y));
    int y=std::get<1>(from_map_point_to_windows_point(goal_x,goal_y));
    SDL_Rect rect = {x-Aim_w/2,y-Aim_h/2,Aim_w, Aim_h};
    SDL_RenderDrawRect(renderer, &rect);
    }
}
