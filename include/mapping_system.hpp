#pragma once
#include <opencv2/opencv.hpp>
#include <vector>
#include <string>

/**
 * @brief 系统配置参数
 * @details 存储物理参数和映射参数
 */
struct SystemConfig {
    double install_height;        // 相机安装高度 (mm)
    double speed;                 // 传送带速度 (m/s)
    double fps;                   // 相机帧率 (帧/秒)
    int px_per_m;                 // 像素密度 (像素/米)
    double rho;                   // 物料密度 (kg/m³)
    int dem_rows;                 // 高程图行数 (横向分辨率)
    int dem_cols;                 // 高程图列数 (纵向最大长度)
    double min_object_height_m;   // 最小物体高度阈值 (米)
    int ply_stride;               // PLY点云采样步长
};

/**
 * @brief 3D点坐标
 */
struct Point3D {
    float x, y, z;  // 世界坐标系坐标 (米)
};

/**
 * @brief 映射系统 - 线扫描3D重建核心
 * @details 将深度图序列拼接为全局高程图(DEM)，并生成点云
 */
class MappingSystem {
public:
    /**
     * @brief 构造函数
     * @param config_path 配置文件路径
     * @details 加载配置并初始化CUDA (如果可用)
     */
    MappingSystem(const std::string& config_path);

    /**
     * @brief 重置映射状态
     * @details 清空高程图，重置拼接位置
     */
    void reset();

    /**
     * @brief 处理单帧深度图
     * @param depth_img_16u 深度图 (CV_16UC1, 单位: mm)
     * @param frame_idx 帧索引 (从1开始)
     * @details 旋转、切片、缩放后拼接到全局高程图
     */
    void process_frame(const cv::Mat& depth_img_16u, int frame_idx);

    /**
     * @brief 批量处理深度图
     * @param depth_batch 深度图批次
     * @param frame_ids 帧索引列表
     * @details 批量拼接，提高效率
     */
    void process_frame_batch(const std::vector<cv::Mat>& depth_batch, const std::vector<int>& frame_ids);

    /**
     * @brief 生成完整PLY点云文件
     * @param batch_id 批次编号
     * @param output_dir 输出目录
     * @return std::string PLY文件路径
     * @details 从高程图生成无断层点云，按stride采样
     */
    std::string generate_full_ply(int batch_id, const std::string& output_dir = "output_ply");

    /**
     * @brief 计算全局体积和质量
     * @param out_mass 输出质量 (kg)
     * @return double 体积 (m³)
     * @details 基于高程图积分计算
     */
    double calculate_global_volume(double& out_mass);

    /**
     * @brief 获取高程图
     * @return cv::Mat 高程图 (CV_32FC1, 单位: 米)
     */
    cv::Mat get_dem_map() const;

    /**
     * @brief 获取拼接长度
     * @return double 物理长度 (米)
     */
    double get_stitched_length_m() const;

private:
    SystemConfig config;       // 系统配置
    bool cuda_enabled = false; // CUDA是否启用
    cv::Mat dem_matrix;        // 全局高程图 (CV_32FC1, 单位: 米)
    int max_end_px = 0;        // 当前拼接到的像素列位置
};
