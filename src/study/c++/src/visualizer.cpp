// visualizer.cpp
#include "../include/visualizer.h"
#include <string>
#include <functional>
#include <cmath>
#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>
#include <SDL2/SDL_image.h>
#include <chrono>
#include <thread>
Visualizer::Visualizer(int w, int h) : width(w), height(h) {
    SDL_Init(SDL_INIT_VIDEO);
    TTF_Init();
    window = SDL_CreateWindow(
        "CarlaAutoDrive",
        SDL_WINDOWPOS_CENTERED,
        SDL_WINDOWPOS_CENTERED,
        width, height,
        0
    );
    renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
    font = TTF_OpenFont("/usr/share/fonts/opentype/noto/NotoSerifCJK-Bold.ttc", 18);  // 替换为你的字体路径
    if (!font) {
        SDL_Log("无法加载字体: %s", TTF_GetError());
    }
    resetButton1= {
        20, 120,
        80, 30,
        "重置仿真器",
        false,
        false,
        false,
        [](){}
    };
    resetButton2= {
        220, 540,
        80, 30,
        "导航视图",
        false,
        false,
        false,
        [](){}
    };
    
    resetButton3= {
        220, 300,
        80, 30,
        "精准定位",
        false,
        false,
        false,
        [](){}
    };
    resetButton4= {
        320, 300,
        80, 30,
        "算法定位",
        false,
        false,
        false,
        [](){}
    };
    resetButton5= {
        420, 300,
        80, 30,
        "标点选取",
        false,
        false,
        false,
        [](){}
    };
    resetButton6= {
        420, 340,
        80, 30,
        "清除标点",
        false,
        false,
        false,
        [](){}
    };
}
void Visualizer::update_car(float pose_right_x,float pose_right_y,float angle){
    ego_car.pose_right_x=pose_right_x;
    ego_car.pose_right_y=pose_right_y;
    ego_car.angle=angle;
}

std::tuple<int,int>Visualizer::from_map_point_to_windows_point(float x,float y){
    int result_x=int((x+5-map_pos_start_x)/0.3+moveX+220);
    int result_y=int(((-y+5-map_pos_start_y)/0.3+moveY));
    return{result_x,result_y};

}
std::tuple<float,float>Visualizer::from_windows_point_to_map_point(int x,int y){
    float result_x=-moveX*0.3+x+map_pos_start_x-5;
    float result_y=-(-moveY*0.3+y+map_pos_start_y-5);
    return{result_x,result_y};
}

// 释放资源
Visualizer::~Visualizer() {
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    TTF_Quit();
    SDL_Quit();
}
void Visualizer::drawText(int x, int y, const std::string& text) {
    if (!font) {
        return;
    }

    SDL_Color textColor = {200, 255, 200, 255};

    SDL_Surface* textSurface = TTF_RenderUTF8_Blended(font, text.c_str(), textColor);
    if (!textSurface) {
        SDL_Log("文字渲染失败: %s", TTF_GetError());
        return;
    }
    SDL_Texture* textTexture = SDL_CreateTextureFromSurface(renderer, textSurface);
    if (!textTexture) {
        SDL_Log("纹理创建失败: %s", SDL_GetError());
        SDL_FreeSurface(textSurface);  // 释放表面
        return;
    }

    SDL_Rect renderQuad = {
        x, y,
        textSurface->w,
        textSurface->h
    };

    SDL_RenderCopy(renderer, textTexture, NULL, &renderQuad);

    SDL_DestroyTexture(textTexture);
    SDL_FreeSurface(textSurface);
}

void Visualizer::drawButtons() {
    if(resetButton1.is_clicked==1){SDL_SetRenderDrawColor(renderer,100,100,100,255);}
    else if (resetButton1.is_hovered==1){SDL_SetRenderDrawColor(renderer,60,100,100,255);}
    else{SDL_SetRenderDrawColor(renderer,20,100,100,255);}
    SDL_Rect buttonRect1 = {
        resetButton1.x, resetButton1.y,
        resetButton1.width, resetButton1.height
    };
    SDL_RenderFillRect(renderer, &buttonRect1);  // 填充按钮
    if(resetButton2.is_clicked==1){SDL_SetRenderDrawColor(renderer,100,100,100,255);}
    else if (resetButton2.is_hovered==1){SDL_SetRenderDrawColor(renderer,60,100,100,255);}
    else{SDL_SetRenderDrawColor(renderer,20,100,100,255);}
    SDL_Rect buttonRect2 = {
        resetButton2.x, resetButton2.y,
        resetButton2.width, resetButton2.height
    };
    SDL_RenderFillRect(renderer, &buttonRect2);  // 填充按钮
    SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);  // 白色边框
    drawText(resetButton1.x, resetButton1.y, resetButton1.text);  // 绘制文字
    drawText(resetButton2.x, resetButton2.y, resetButton2.text);  // 绘制文字
    if(resetButton2.is_open==true){
        if(resetButton3.is_clicked==1){SDL_SetRenderDrawColor(renderer,100,100,100,255);}
        else if (resetButton3.is_hovered==1){SDL_SetRenderDrawColor(renderer,60,100,100,255);}
        else{SDL_SetRenderDrawColor(renderer,20,100,100,255);}
            SDL_Rect buttonRect3 = {
            resetButton3.x, resetButton3.y,
            resetButton3.width, resetButton3.height
        };
        SDL_RenderFillRect(renderer, &buttonRect3);  // 填充按钮
        if(resetButton4.is_clicked==1){SDL_SetRenderDrawColor(renderer,100,100,100,255);}
        else if (resetButton4.is_hovered==1){SDL_SetRenderDrawColor(renderer,60,100,100,255);}
        else{SDL_SetRenderDrawColor(renderer,20,100,100,255);}
            SDL_Rect buttonRect4 = {
            resetButton4.x, resetButton4.y,
            resetButton4.width, resetButton4.height
        };
        SDL_RenderFillRect(renderer, &buttonRect4);  // 填充按钮
        if(resetButton5.is_clicked==1){SDL_SetRenderDrawColor(renderer,100,100,100,255);}
        else if (resetButton5.is_open==true){SDL_SetRenderDrawColor(renderer,180,100,100,255);}
        else if (resetButton5.is_hovered==1){SDL_SetRenderDrawColor(renderer,60,100,100,255);}

        else{SDL_SetRenderDrawColor(renderer,20,100,100,255);}
            SDL_Rect buttonRect5 = {
            resetButton5.x, resetButton5.y,
            resetButton5.width, resetButton5.height
        };
        SDL_RenderFillRect(renderer, &buttonRect5);  // 填充按钮
        if(resetButton6.is_clicked==1){SDL_SetRenderDrawColor(renderer,100,100,100,255);}
        else if (resetButton6.is_hovered==1){SDL_SetRenderDrawColor(renderer,60,100,100,255);}
        else{SDL_SetRenderDrawColor(renderer,20,100,100,255);}
            SDL_Rect buttonRect6 = {
            resetButton6.x, resetButton6.y,
            resetButton6.width, resetButton6.height
        };
        SDL_RenderFillRect(renderer, &buttonRect6);  // 填充按钮
    drawText(resetButton3.x, resetButton3.y, resetButton3.text);
    drawText(resetButton4.x, resetButton4.y, resetButton4.text);
    drawText(resetButton5.x, resetButton5.y, resetButton5.text);
    drawText(resetButton6.x, resetButton6.y, resetButton6.text);
    };
}
void Visualizer::drawWindows(){
    SDL_SetRenderDrawColor(renderer, 20, 20, 150, 255);
    SDL_Rect rect1 = {220, 0, 520, 570};  // x, y, 宽, 高
    SDL_RenderDrawRect(renderer, &rect1);  // 空心矩形
    SDL_Rect rect2 = {740, 0, 220, 570};  // x, y, 宽, 高
    SDL_RenderDrawRect(renderer, &rect2);  // 空心矩形
    SDL_Rect rect3 = {0, 0, 220, 570};  // x, y, 宽, 高
    SDL_RenderDrawRect(renderer, &rect3);  // 空心矩形
    SDL_Rect rect4 = {0, 570, 960, 150};  // x, y, 宽, 高
    SDL_RenderDrawRect(renderer, &rect4);  // 空心矩形
    SDL_Rect rect5 = {220, 540, 520, 30};  // x, y, 宽, 高
    SDL_RenderDrawRect(renderer, &rect5);  // 空心矩形
}

// 刷新窗口显示
void Visualizer::draw_words(){
    drawText(810,0,"规划模块");
    drawText(80,0,"感知模块");
    drawText(200,600,"车辆速度:");
    drawText(200,640,"转向:");
    drawText(300,600,std::to_string(int(ego_car.speed*3.6))+"km/h");
    if (resetButton2.is_open==true){
        drawText(220,370,"小车精确坐标(X: "+std::to_string(ego_car.pose_right_x)+",Y: "+std::to_string(ego_car.pose_right_y)+")");
        drawText(220,400,"鼠标位置(X: "+std::to_string((mouse_x-moveX)*0.3+map_pos_start_x-5)+",Y: "+std::to_string(-((mouse_y-moveY)*0.3+map_pos_start_y-5))+")");
    }

}

void Visualizer::refresh() {
    SDL_SetRenderDrawColor(renderer, 30, 30, 50, 10);
    SDL_RenderClear(renderer);
    drawWindows();
    drawButtons();
    draw_words();
    if(resetButton2.is_open==true){
        if(button2andy<300){
            button2andy+=5;
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        draw_map(button2andy,true);
        drawCar();
    }
    else if(resetButton2.is_open==false){
        if(button2andy>0){
            button2andy-=5;
            std::this_thread::sleep_for(std::chrono::milliseconds(1));}
        draw_map(button2andy,false);
    };
    SDL_RenderPresent(renderer);
}

bool Visualizer::handleEvents() {
    SDL_Event event;
    int mouseX, mouseY;
    SDL_GetMouseState(&mouseX, &mouseY);  // 获取鼠标位置

    // 检测鼠标是否在按钮上
    resetButton1.is_hovered = (
        mouseX >= resetButton1.x && mouseX <= resetButton1.x + resetButton1.width &&
        mouseY >= resetButton1.y && mouseY <= resetButton1.y + resetButton1.height
    );
    resetButton2.is_hovered = (
        mouseX >= resetButton2.x && mouseX <= resetButton2.x + resetButton2.width &&
        mouseY >= resetButton2.y && mouseY <= resetButton2.y + resetButton2.height
    );
    resetButton3.is_hovered = (
        mouseX >= resetButton3.x && mouseX <= resetButton3.x + resetButton3.width &&
        mouseY >= resetButton3.y && mouseY <= resetButton3.y + resetButton3.height
    );
    resetButton4.is_hovered = (
        mouseX >= resetButton4.x && mouseX <= resetButton4.x + resetButton4.width &&
        mouseY >= resetButton4.y && mouseY <= resetButton4.y + resetButton4.height
    );
    resetButton5.is_hovered = (
        mouseX >= resetButton5.x && mouseX <= resetButton5.x + resetButton5.width &&
        mouseY >= resetButton5.y && mouseY <= resetButton5.y + resetButton5.height
    );
    resetButton6.is_hovered = (
        mouseX >= resetButton6.x && mouseX <= resetButton6.x + resetButton6.width &&
        mouseY >= resetButton6.y && mouseY <= resetButton6.y + resetButton6.height
    );
    while (SDL_PollEvent(&event)) {
        if (event.type == SDL_QUIT) {
            return false; // 退出程序
        }
        if (event.type == SDL_MOUSEBUTTONDOWN && event.button.button == SDL_BUTTON_LEFT) {
            if (resetButton1.is_hovered) {
                resetButton1.is_clicked = true;
                resetButton1.onClick();  // 触发按钮点击回调
            }
            if (resetButton2.is_hovered) {
                resetButton2.is_clicked = true;
                resetButton2.onClick();  // 触发按钮点击回调
                if (resetButton2.is_open==false){resetButton2.is_open = true;button2andy=0;}
                else if (resetButton2.is_open==true){resetButton2.is_open = false;button2andy=300;}
            }
            if (resetButton3.is_hovered) {
                resetButton3.is_clicked = true;
                resetButton3.onClick();  // 触发按钮点击回调
                if (resetButton3.is_open==false){resetButton3.is_open = true;}
                else if (resetButton3.is_open==true){resetButton3.is_open = false;}

            }
            if (resetButton4.is_hovered) {
                resetButton4.is_clicked = true;
                resetButton4.onClick();  // 触发按钮点击回调
            }
            if (resetButton5.is_hovered) {
                resetButton5.is_clicked = true;
            }
            if (resetButton6.is_hovered) {
                resetButton6.is_clicked = true;
                goal_x=0;
                goal_y=0;
                resetButton5.is_open=false;
            }
            if (resetButton2.is_open==true){
                startX = event.button.x;
                startY = event.button.y;
                if (startX>222&&startX<738&&startY>2&&startY<298){is_move=true;}
            }
            if (mouseX>220&&mouseX<740&&mouseY>0&&mouseY<300&&resetButton2.is_open==true&&resetButton5.is_open==true){
                goal_x=(mouse_x-moveX)*0.3+map_pos_start_x-5;
                goal_y=-((mouse_y-moveY)*0.3+map_pos_start_y-5);
                resetButton5.onClick();
            }
        }
        if (event.type == SDL_MOUSEBUTTONUP && event.button.button == SDL_BUTTON_LEFT) {
            resetButton1.is_clicked = false;  // 松开后取消高亮
            resetButton2.is_clicked = false;  // 松开后取消高亮
            resetButton3.is_clicked = false;  // 松开后取消高亮
            resetButton4.is_clicked = false;  // 松开后取消高亮
            resetButton5.is_clicked = false;  // 松开后取消高亮
            resetButton6.is_clicked = false;  // 松开后取消高亮
            if (resetButton5.is_hovered) {
                if (resetButton5.is_open==false){resetButton5.is_open= true;}
                else{resetButton5.is_open=false;}
            }
            is_move=false;
            lastX=moveX;
            lastY=moveY;
        }
    }
    if (is_move==true){
        moveX=(mouseX-startX+lastX);
        moveY=(mouseY-startY+lastY);
    }
    if (resetButton2.is_open==true){
        if(resetButton3.is_open==true){
            int x=std::get<0>(from_map_point_to_windows_point(ego_car.pose_right_x,ego_car.pose_right_y));
            int y=std::get<1>(from_map_point_to_windows_point(ego_car.pose_right_x,ego_car.pose_right_y));
            int change_x=x-520;
            int change_y=y-150;
            moveX+=-change_x;
            moveY+=-change_y;
        }
        if (moveX>0){
           moveX=0;
        }
        else if (moveX<-map_width+520){
            moveX=-map_width+520;
        }
        if (moveY>0){
           moveY=0;
        }
        else if (moveY<-map_height+300){
           moveY=-map_height+300;
        }
    }
    if (mouseX>220&&mouseX<740&&mouseY>0&&mouseY<300&&resetButton2.is_open==true){
        mouse_x=mouseX-220;
        mouse_y=mouseY;
    }
    else{
        mouse_x=0;
        mouse_y=0;
    }
    return true; // 继续运行
}
void Visualizer::setResetButtonCallback1(std::function<void()> callback) {
    resetButton1.onClick = callback;
}
void Visualizer::setResetButtonCallback5(std::function<void()> callback) {
    resetButton5.onClick = callback;
}
