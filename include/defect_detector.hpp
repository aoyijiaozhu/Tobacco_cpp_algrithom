#ifndef DEFECT_DETECTOR_HPP
#define DEFECT_DETECTOR_HPP

#include <opencv2/opencv.hpp>
#include "json.hpp"

using json = nlohmann::json;
using namespace cv;

/**
 * @brief 缺陷检测器
 * @details 基于高程图检测凹坑和凸起缺陷，支持CPU和CUDA加速
 */
class DefectDetector {
public:
    /**
     * @brief 构造函数
     * @param config_path 配置文件路径
     * @details 加载检测参数并初始化CUDA (如果可用)
     */
    DefectDetector(const std::string& config_path);

    /**
     * @brief 全局缺陷检测
     * @param dem_map 高程图 (CV_32FC1, 单位: 米)
     * @return json 缺陷列表 (JSON数组)
     * @details 自动选择CPU或CUDA实现
     */
    json detect_global(const Mat& dem_map);

private:
    /**
     * @brief CPU版本缺陷检测
     * @param dem_map 高程图
     * @return json 缺陷列表
     */
    json detect_global_cpu(const Mat& dem_map);

#if defined(HAVE_OPENCV_CUDAIMGPROC) && defined(HAVE_OPENCV_CUDAFILTERS) && defined(HAVE_OPENCV_CUDAARITHM)
    /**
     * @brief CUDA版本缺陷检测
     * @param dem_map 高程图
     * @return json 缺陷列表
     */
    json detect_global_cuda(const Mat& dem_map);
#endif

    bool cuda_enabled;            // CUDA是否启用
    double defect_thresh_mm;      // 缺陷阈值 (mm)
    double min_long_mm;           // 最小长轴长度 (mm)
    double min_short_mm;          // 最小短轴长度 (mm)
    int margin_x;                 // X方向边缘裕度 (像素)
    int margin_y;                 // Y方向边缘裕度 (像素)
    int bg_kernel_width;          // 背景滤波核宽度 (像素)
    int morph_kernel_size;        // 形态学操作核大小 (像素)
    double max_valid_height_mm;   // 最大有效高度 (mm)
    double min_valid_height_mm;   // 最小有效高度 (mm)
    bool draw_filtered;           // 是否绘制过滤后的结果
};

#endif
