// visualizer.h
#ifndef VISUALIZER_H
#define VISUALIZER_H
#include <vector>
#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>
#include <string>
#include <functional>
#include "car.h"
struct Button {
    int x, y;           // 按钮位置（左上角）
    int width, height;  // 按钮尺寸
    std::string text;   // 按钮文字
    bool is_hovered;    // 是否鼠标悬停
    bool is_clicked;
    bool is_open;
    std::function<void()> onClick;  // 点击回调函数
};


class Visualizer {
private:
    SDL_Window* window;
    SDL_Renderer* renderer;
    int width, height;
    int button2andy=0;
    Button resetButton1;
    Button resetButton2;
    Button resetButton3;
    Button resetButton4;
    Button resetButton5;
    Button resetButton6;
    Button resetButton7;
    Button resetButton8;
    TTF_Font* font;
    void drawText(int x, int y, const std::string& text);
public:
    Visualizer(int w = 1600, int h = 1200);
    ~Visualizer();
    Car ego_car;
    void drawButtons();
    void drawWindows();
    void draw_words();
    // 刷新窗口
    void refresh();
    // 处理事件（如关闭窗口）
    bool handleEvents();
    bool connect_situation=false;
    bool is_move=false;
    int startX;
    int startY;
    int lastX=0;
    int lastY=0;
    int moveX=0;
    int moveY=0;
    int mouse_x=0;
    int mouse_y=0;
    int map_width=0;
    int map_height=0;
    int goal_x=0;
    int goal_y=0;
    float map_pos_start_x=0;
    float map_pos_start_y=0;
    void setResetButtonCallback1(std::function<void()> callback);
    void setResetButtonCallback5(std::function<void()> callback);
    void update_car(float pose_right_x,float pose_right_y,float angle);
    void draw_map(int button2andy,bool show_situation);
    void drawCar();
    void drawAim();
    std::tuple<int,int>from_map_point_to_windows_point(float x,float y);
    std::tuple<float,float>from_windows_point_to_map_point(int x,int y);
    void update_map_wh(int w,int h){
        map_width=w;
        map_height=h;
    }
    SDL_Texture* img_texture = nullptr;
    void setImageData(int img_w,int img_h,const std::vector<uint8_t>& rgba_data);
    void drawImage(int x,int y);
    int img_width = 0;
    int img_height = 0;
};


#endif
