#pragma once 
#include <iostream>

class CoordinateLocator {
private:
    double speed;
    double fps;
    double h_install;
    double dist_per_frame;

public:
    // 构造函数 (默认参数更新为你的配置)
    CoordinateLocator(double speed = 0.132, double fps = 5.0, double h_install = 2.7);

    // [保留] 转换函数: 返回 X, Y (简单版/旧版)
    void get_world_coordinates(int frame_id, int u, int v, double& out_x, double& out_y);

    // [新增] 3D 点云专用的局部到全局转换
    // 输入: frame_id(帧号), local_x/y/z (针孔相机模型反解出的局部坐标，单位米)
    // 输出: world_x/y/z (物理世界绝对坐标，单位米)
    void local_to_world(int frame_id, double local_x, double local_y, double local_z, 
                        double& world_x, double& world_y, double& world_z);
    
    // [新增] 仅获取当前的全局X坐标基准 (用于异常日志的快速定位)
    double get_base_x(int frame_id);
};