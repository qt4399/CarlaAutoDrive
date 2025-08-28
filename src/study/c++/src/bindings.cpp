// bindings.cpp
#include <pybind11/pybind11.h>
#include <pybind11/functional.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>
#include <SDL2/SDL_image.h>
#include "../include/visualizer.h"

namespace py = pybind11;
py::list get_predict_x_list(const Car& car) {
    py::list result;
    for (int i = 0; i < 15; ++i) {
        result.append(car.predict_x[i]);
    }
    return result;
}
py::list get_predict_y_list(const Car& car) {
    py::list result;
    for (int i = 0; i < 15; ++i) {
        result.append(car.predict_y[i]);
    }
    return result;
}
PYBIND11_MODULE(autoviz, m) {
    m.doc() = "自动驾驶可视化Python接口";
    py::class_<Visualizer>(m, "Visualizer")
        .def(py::init<int, int>(), py::arg("width") = 960, py::arg("height") = 720)
        .def("update_map_wh", &Visualizer::update_map_wh,
             py::arg("w"), py::arg("h"),
             "更新地图参数")
        .def("update_car", &Visualizer::update_car,
             py::arg("pose_right_x"),py::arg("pose_right_y"),py::arg("angle"),
             "更新车辆参数")
        .def_readwrite("moveX", &Visualizer::moveX)
        .def_readwrite("moveY", &Visualizer::moveY)
        .def_readwrite("map_pos_start_x", &Visualizer::map_pos_start_x)
        .def_readwrite("map_pos_start_y", &Visualizer::map_pos_start_y)
        .def_readwrite("goal_x", &Visualizer::goal_x)
        .def_readwrite("goal_y", &Visualizer::goal_y)
        .def_readwrite("ego_car", &Visualizer::ego_car)
        .def("refresh", &Visualizer::refresh, "刷新窗口")
        .def("handle_events", &Visualizer::handleEvents,
             "处理窗口事件，返回False表示需要退出")
        .def("set_reset_callback1", &Visualizer::setResetButtonCallback1,
             "设置重置按钮的点击回调函数")
        .def("set_goal_callback", &Visualizer::setResetButtonCallback5,
             "设置目标点回调函数")
        .def("set_image_data", &Visualizer::setImageData,
             py::arg("img_width"), py::arg("img_height"), py::arg("gray_data"),
             "传递灰度图数据给C++（img_width：图片宽；img_height：图片高；gray_data：灰度格式像素数组，1维列表）");
    py::class_<Car>(m, "Car")
        .def(py::init<>())
        .def_readwrite("global_path_num", &Car::global_path_num)
        .def_readwrite("steer", &Car::steer)
        .def_readwrite("speed", &Car::speed)
        .def_readwrite("power", &Car::power)
        .def_readwrite("AutoDriveOn", &Car::AutoDriveOn)
        .def("get_predict_x", &get_predict_x_list, "获取predict_x数组（返回列表）")
        .def("get_predict_y", &get_predict_y_list, "获取predict_y数组（返回列表）")
        .def("set_global_x", &Car::set_global_x,
         py::arg("index"), py::arg("value"),  // 2个参数 → 2个注解
         "修改 global_x 数组的指定元素（index：0-999，value：新值）")
         .def("set_global_y", &Car::set_global_y,
         py::arg("index"), py::arg("value"),  // 2个参数 → 2个注解
         "修改 global_y 数组的指定元素（index：0-999，value：新值）")
        .def("PlanAndControl", &Car::PlanAndControl, "刷新窗口");



}

