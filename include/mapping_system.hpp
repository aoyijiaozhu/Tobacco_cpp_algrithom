/**
 * @file mapping_system.hpp
 * @brief 线扫描拼接系统 -- 将连续深度帧拼接为全局 DEM，并提供 PLY 导出和体积计算
 */
#pragma once
#include <opencv2/opencv.hpp>
#include <vector>
#include <string>

/**
 * 系统配置参数，从 config.json 的 system 和 mapping 段读取
 */
struct SystemConfig {
    double install_height;       // 传感器安装高度（mm）
    double speed;                // 传送带速度（m/s）
    double fps;                  // 采集帧率
    int px_per_m;                // DEM 分辨率：每米像素数
    double rho;                  // 物料密度（kg/m3），用于质量估算
    int dem_rows;                // DEM 矩阵行数（垂直于传送带方向）
    int dem_cols;                // DEM 矩阵最大列数（传送带方向）
    double min_object_height_m;  // 最小有效物体高度（m），低于此高度不计入体积
    int ply_stride;              // PLY 导出降采样步长
};

/**
 * 三维点，用于 PLY 导出时的临时存储
 */
struct Point3D {
    float x, y, z;
};

/**
 * 线扫描拼接系统
 *
 * 将传送带上连续采集的深度帧，按物理速度逐帧拼接到全局 DEM（数字高程模型）中。
 * 拼接完成后可导出 PLY 点云、计算体积/质量、获取 DEM 供缺陷检测使用。
 */
class MappingSystem {
public:
    /**
     * @brief 构造函数，从 config.json 加载配置并初始化 DEM 矩阵
     * @param config_path 配置文件路径
     */
    MappingSystem(const std::string& config_path);

    /**
     * @brief 重置拼接状态，清空 DEM 矩阵和拼接进度
     */
    void reset();

    /**
     * @brief 处理单帧深度图，拼接到全局 DEM
     * @param depth_img_16u 16 位无符号深度图（CV_16UC1，640x360）
     * @param frame_idx 帧序号（从 1 开始），用于计算该帧在 DEM 中的绝对像素位置
     */
    void process_frame(const cv::Mat& depth_img_16u, int frame_idx);

    /**
     * @brief 批量处理多帧深度图，CUDA 可用时走 GPU 路径
     * @param depth_batch 深度图批次
     * @param frame_ids 对应的帧序号
     */
    void process_frame_batch(const std::vector<cv::Mat>& depth_batch, const std::vector<int>& frame_ids);

    /**
     * @brief 从拼接好的 DEM 导出 PLY 点云文件
     * @param batch_id 批次号，用于文件命名
     * @param output_dir 输出目录，默认 "output_ply"
     * @return 生成的 PLY 文件路径
     */
    std::string generate_full_ply(int batch_id, const std::string& output_dir = "output_ply");

    /**
     * @brief 对 DEM 进行面积积分，计算全局体积和质量
     * @param out_mass [输出] 估算质量（kg），= 体积 x 物料密度
     * @return 体积（m3）
     */
    double calculate_global_volume(double& out_mass);

    /**
     * @brief 获取当前 DEM 有效区域的副本，供缺陷检测使用
     * @return DEM 矩阵（CV_32FC1，单位：米）
     */
    cv::Mat get_dem_map() const;

    /**
     * @brief 获取当前拼接长度
     * @return 拼接长度（米）
     */
    double get_stitched_length_m() const;

private:
    SystemConfig config;         // 系统配置
    bool cuda_enabled = false;   // 是否启用 CUDA 加速
    cv::Mat dem_matrix;          // 全局 DEM 矩阵（CV_32FC1），存储物理高度（米）
    int max_end_px = 0;          // 当前已拼接到的像素列位置
};
