#include "locator.hpp"

CoordinateLocator::CoordinateLocator(double speed, double fps, double h_install)
    : speed(speed), fps(fps), h_install(h_install) {
    // 计算每帧移动距离 (米)
    this->dist_per_frame = speed / fps;
}

void CoordinateLocator::get_world_coordinates(int frame_id, int u, int v, double& out_x, double& out_y) {
    // X坐标：基于帧号计算传送带移动距离
    out_x = (frame_id - 1) * this->dist_per_frame;
    // Y坐标：基于像素位置计算横向位置
    double physical_width = 2.0;
    double px_per_m = 640.0 / physical_width;
    out_y = u / px_per_m;
}

void CoordinateLocator::local_to_world(int frame_id, double local_x, double local_y, double local_z,
                                       double& world_x, double& world_y, double& world_z) {
    // X轴：传送带运动方向，基础偏移 + 局部Y延伸
    world_x = (frame_id - 1) * dist_per_frame + local_y;

    // Y轴：传送带横向，相机安装在中心位置(1.0m)
    world_y = 1.0 + local_x;

    // Z轴：垂直高度，安装高度 - 深度值
    world_z = h_install - local_z;
}

double CoordinateLocator::get_base_x(int frame_id) {
    // 返回该帧的全局X坐标基准
    return (frame_id - 1) * dist_per_frame;
}