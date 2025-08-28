cmake_minimum_required(VERSION 3.10)
project(autoviz)

# 寻找Python和SDL2
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
find_package(SDL2 REQUIRED)
set(SDL2_TTF_INCLUDE_DIRS "/usr/include/SDL2")  # 头文件目录
set(SDL2_TTF_LIBRARIES "/usr/lib/x86_64-linux-gnu/libSDL2_ttf.so")  # 库文件路径
# 验证路径是否正确
if(NOT EXISTS ${SDL2_TTF_INCLUDE_DIRS}/SDL_ttf.h)
    message(FATAL_ERROR "SDL2_ttf头文件未找到，请检查路径是否正确")
endif()
if(NOT EXISTS ${SDL2_TTF_LIBRARIES})
    message(FATAL_ERROR "SDL2_ttf库文件未找到，请检查路径是否正确")
endif()

find_path(SDL2_IMAGE_INCLUDE_DIRS SDL_image.h)
find_library(SDL2_IMAGE_LIBRARIES SDL2_image)
# 加入pybind11
add_subdirectory(pybind11)

# 编译C++核心代码
add_library(visualizer_core
            STATIC
            src/visualizer.cpp
            src/map.cpp
            src/vis_picture.cpp
            src/car.cpp
            src/BasePredictAndControll.cpp
            )
set_target_properties(visualizer_core PROPERTIES POSITION_INDEPENDENT_CODE ON)#设置-fpic让代码位置无关

target_include_directories(visualizer_core PUBLIC ${SDL2_INCLUDE_DIRS} ${SDL2_TTF_INCLUDE_DIRS})
target_link_libraries(visualizer_core ${SDL2_LIBRARIES} ${SDL2_TTF_LIBRARIES} ${SDL2_IMAGE_LIBRARIES})

# 生成Python模块
pybind11_add_module(autoviz src/bindings.cpp)
target_link_libraries(autoviz PRIVATE visualizer_core Python3::Python)

