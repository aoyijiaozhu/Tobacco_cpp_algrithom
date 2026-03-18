#pragma once
#include <iostream>

/**
 * @brief 坐标定位器 - 局部坐标到世界坐标转换
 * @details 基于传送带速度和相机参数进行坐标变换
 */
class CoordinateLocator {
private:
    double speed;           // 传送带速度 (m/s)
    double fps;             // 相机帧率 (帧/秒)
    double h_install;       // 相机安装高度 (米)
    double dist_per_frame;  // 每帧移动距离 (米)

public:
    /**
     * @brief 构造函数
     * @param speed 传送带速度 (m/s)，默认 0.132
     * @param fps 相机帧率 (帧/秒)，默认 5.0
     * @param h_install 相机安装高度 (米)，默认 2.7
     */
    CoordinateLocator(double speed = 0.132, double fps = 5.0, double h_install = 2.7);

    /**
     * @brief 获取世界坐标 (简化版)
     * @param frame_id 帧编号
     * @param u 图像u坐标 (像素)
     * @param v 图像v坐标 (像素)
     * @param out_x 输出世界X坐标 (米)
     * @param out_y 输出世界Y坐标 (米)
     */
    void get_world_coordinates(int frame_id, int u, int v, double& out_x, double& out_y);

    /**
     * @brief 局部坐标到世界坐标转换 (3D点云专用)
     * @param frame_id 帧编号
     * @param local_x 局部X坐标 (米)
     * @param local_y 局部Y坐标 (米)
     * @param local_z 局部Z坐标 (米)
     * @param world_x 输出世界X坐标 (米)
     * @param world_y 输出世界Y坐标 (米)
     * @param world_z 输出世界Z坐标 (米)
     * @details 将相机坐标系转换为传送带世界坐标系
     */
    void local_to_world(int frame_id, double local_x, double local_y, double local_z,
                        double& world_x, double& world_y, double& world_z);

    /**
     * @brief 获取帧的全局X坐标基准
     * @param frame_id 帧编号
     * @return double 全局X坐标 (米)
     * @details 用于快速定位缺陷位置
     */
    double get_base_x(int frame_id);
};