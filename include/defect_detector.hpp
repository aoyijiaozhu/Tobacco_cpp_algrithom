/**
 * @file defect_detector.hpp
 * @brief 表面缺陷检测器 -- 基于背景差分法检测 DEM 上的凹坑和凸起缺陷
 */
#ifndef DEFECT_DETECTOR_HPP
#define DEFECT_DETECTOR_HPP

#include <opencv2/opencv.hpp>
#include "json.hpp"

using json = nlohmann::json;
using namespace cv;

/**
 * 缺陷检测器
 *
 * 在全局 DEM 高程图上，通过水平方向 boxFilter 估计背景，
 * 计算与背景的偏差来识别凹坑（pit）和凸起（bump）缺陷。
 * 支持 CPU 和 CUDA 两种计算路径，运行时自动选择。
 */
class DefectDetector {
public:
    /**
     * @brief 构造函数，从 config.json 加载检测参数
     * @param config_path 配置文件路径
     */
    DefectDetector(const std::string& config_path);

    /**
     * @brief 对全局 DEM 执行缺陷检测
     * @param dem_map DEM 高程图（CV_32FC1，单位：米）
     * @return JSON 数组，每个元素描述一个缺陷（type/dev_mm/width_mm/length_mm/center）
     */
    json detect_global(const Mat& dem_map);

private:
    // CPU 检测路径
    json detect_global_cpu(const Mat& dem_map);

#if defined(HAVE_OPENCV_CUDAIMGPROC) && defined(HAVE_OPENCV_CUDAFILTERS) && defined(HAVE_OPENCV_CUDAARITHM)
    // CUDA 加速检测路径
    json detect_global_cuda(const Mat& dem_map);
#endif

    bool cuda_enabled;            // 是否启用 CUDA
    double defect_thresh_mm;      // 缺陷偏差阈值（mm）
    double min_long_mm;           // 缺陷最小长轴尺寸（mm）
    double min_short_mm;          // 缺陷最小短轴尺寸（mm）
    int margin_x;                 // 水平安全边距（px），边缘区域不检测
    int margin_y;                 // 垂直安全边距（px），边缘区域不检测
    int bg_kernel_width;          // 背景估计 boxFilter 核宽度（px），必须为奇数
    int morph_kernel_size;        // 形态学开运算核大小（px），必须为奇数
    double max_valid_height_mm;   // 有效高度上限（mm），超出视为无效
    double min_valid_height_mm;   // 有效高度下限（mm），低于视为噪声
    bool draw_filtered;           // 是否在结果图上绘制被过滤的缺陷框（调试用）
};

#endif
