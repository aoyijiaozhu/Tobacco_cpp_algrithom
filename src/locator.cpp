#include "locator.hpp"

CoordinateLocator::CoordinateLocator(double speed, double fps, double h_install) 
    : speed(speed), fps(fps), h_install(h_install) {
    this->dist_per_frame = speed / fps;
}

void CoordinateLocator::get_world_coordinates(int frame_id, int u, int v, double& out_x, double& out_y) {
    out_x = (frame_id - 1) * this->dist_per_frame;
    double physical_width = 2.0; 
    double px_per_m = 640.0 / physical_width;
    out_y = u / px_per_m; 
}

// [核心实现] 局部到全局的 3D 映射
void CoordinateLocator::local_to_world(int frame_id, double local_x, double local_y, double local_z, 
                                       double& world_x, double& world_y, double& world_z) {
    // 1. X轴 (长轴 20m): 基础偏移量 + 图像在长轴方向的视野延伸 (local_y)
    world_x = (frame_id - 1) * dist_per_frame + local_y;
    
    // 2. Y轴 (宽轴 2m): 假设相机安装在 2m 宽度的正中心(即 1.0m 处)
    // 局部坐标 local_x 的中心为 0，向两侧延伸
    world_y = 1.0 + local_x;
    
    // 3. Z轴 (高度): 假设底面(皮带面) Z=0
    // 深度(local_z)越小，说明烟叶堆得越高
    world_z = h_install - local_z;
}

double CoordinateLocator::get_base_x(int frame_id) {
    return (frame_id - 1) * dist_per_frame;
}