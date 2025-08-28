#include "../include/visualizer.h"
#include <string>
#include <functional>
#include <cmath>
#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>
#include <SDL2/SDL_image.h>
#include <chrono>
#include <thread>
void Visualizer::setImageData(int img_w, int img_h, const std::vector<uint8_t>& gray_data) {
    if (img_texture != nullptr) {
        SDL_DestroyTexture(img_texture);
        img_texture = nullptr;
    }
    img_width = img_w;
    img_height = img_h;
    img_texture = SDL_CreateTexture(
        renderer,
        SDL_PIXELFORMAT_RGBA8888,
        SDL_TEXTUREACCESS_STREAMING,
        img_w, img_h
    );
    if (!img_texture) {
        SDL_Log("创建图片纹理失败: %s", SDL_GetError());
        return;
    }
    std::vector<uint8_t> rgba_data(img_w * img_h * 4);
    for (int i = 0; i < img_w * img_h; i++) {
        uint8_t gray_value = gray_data[i];
        rgba_data[i * 4] = gray_value;     // R
        rgba_data[i * 4 + 1] = gray_value; // G
        rgba_data[i * 4 + 2] = gray_value; // B
        rgba_data[i * 4 + 3] = 255;        // A (完全不透明)
    }
    SDL_UpdateTexture(
        img_texture,
        nullptr,
        rgba_data.data(),
        img_w * 4
    );
    SDL_SetTextureBlendMode(img_texture, SDL_BLENDMODE_BLEND);
}

void Visualizer::drawImage(int x, int y) {
    if (!img_texture) {
        SDL_Log("图片纹理未初始化，无法绘制");
        return;
    }
    SDL_Rect img_rect = {
        x,
        y,
        img_width,
        img_height
    };
    SDL_RenderCopy(renderer, img_texture, nullptr, &img_rect);
}