/**
 * @file defect_detector.cpp
 * @brief DefectDetector 实现 -- 基于背景差分的凹坑/凸起缺陷检测，支持 CPU 和 CUDA
 */
#include "defect_detector.hpp"
#include <vector>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <filesystem>
#if defined(HAVE_OPENCV_CUDAIMGPROC) && defined(HAVE_OPENCV_CUDAFILTERS) && defined(HAVE_OPENCV_CUDAARITHM)
#include <opencv2/core/cuda.hpp>
#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudafilters.hpp>
#include <opencv2/cudaimgproc.hpp>
#endif

using namespace cv;
using namespace std;
namespace fs = std::filesystem;
using json = nlohmann::json;

namespace {

/**
 * 从二值掩码中提取缺陷轮廓，过滤后生成缺陷事件列表，同时保存可视化标注图
 *
 * @param height_mm   毫米高程图，用于生成可视化底图
 * @param pit_mask    凹坑二值掩码
 * @param bump_mask   凸起二值掩码
 * @param diff_pit    凹坑偏差图（背景 - 高度）
 * @param diff_bump   凸起偏差图（高度 - 背景）
 * @param defect_thresh_mm  偏差阈值（mm）
 * @param min_long_mm       最小长轴尺寸（mm）
 * @param min_short_mm      最小短轴尺寸（mm）
 * @param draw_filtered     是否绘制被过滤的缺陷框
 * @return JSON 缺陷事件数组
 */
json build_event_list(const Mat& height_mm, Mat pit_mask, Mat bump_mask,
                      const Mat& diff_pit, const Mat& diff_bump,
                      double defect_thresh_mm, double min_long_mm,
                      double min_short_mm, bool draw_filtered) {
    json event_list = json::array();

    // 生成可视化底图：高程图归一化到 0-255 灰度，再转 BGR
    Mat vis_img;
    normalize(height_mm, vis_img, 0, 255, NORM_MINMAX, CV_8UC1);
    cvtColor(vis_img, vis_img, COLOR_GRAY2BGR);

    // 轮廓提取和过滤的通用逻辑
    auto process_contours = [&](Mat& mask, const Mat& diff, const string& type) {
        vector<vector<Point>> contours;
        findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        for (size_t i = 0; i < contours.size(); i++) {
            Rect r = boundingRect(contours[i]);
            // DEM 分辨率为 px_per_m=100，即 1px = 10mm
            double length_mm = r.width * 10.0;
            double width_mm = r.height * 10.0;

            // 在掩码区域内找偏差峰值
            double max_v = 0;
            minMaxLoc(diff(r), nullptr, &max_v, nullptr, nullptr, mask(r));

            bool check_depth = (max_v >= defect_thresh_mm);
            bool check_size = (std::max(length_mm, width_mm) >= min_long_mm) &&
                              (std::min(length_mm, width_mm) >= min_short_mm);

            if (check_depth && check_size) {
                // 同时满足深度和尺寸阈值：记录缺陷事件
                json evt;
                evt["type"] = type;
                evt["dev_mm"] = (type == "pit" ? -max_v : max_v);
                evt["width_mm"] = width_mm;
                evt["length_mm"] = length_mm;
                // 像素坐标转物理坐标（米）
                evt["center_x_m"] = (r.x + r.width / 2.0) * 0.01;
                evt["center_y_m"] = (r.y + r.height / 2.0) * 0.01 - 1.0;
                event_list.push_back(evt);

                // 在可视化图上标注：凹坑红框，凸起黄框
                Scalar color = (type == "pit") ? Scalar(0, 0, 255) : Scalar(0, 255, 255);
                rectangle(vis_img, r, color, 3);
                string label = type + " " + to_string((int)max_v) + "mm";
                putText(vis_img, label, Point(r.x, r.y - 10), FONT_HERSHEY_SIMPLEX, 0.8, color, 2);
            } else if (max_v >= defect_thresh_mm) {
                // 满足深度但不满足尺寸：打印过滤日志
                cout << "[Filtered] " << type << " -> Depth:" << (int)max_v
                     << "mm, W:" << (int)width_mm << ", L:" << (int)length_mm
                     << " | Killed by size threshold" << endl;
                if (draw_filtered) {
                    Scalar color = Scalar(255, 0, 0);
                    rectangle(vis_img, r, color, 2);
                    string label = "filtered " + type + " " + to_string((int)max_v) + "mm";
                    putText(vis_img, label, Point(r.x, r.y - 10), FONT_HERSHEY_SIMPLEX, 0.6, color, 2);
                }
            }
        }
    };

    // 分别处理凹坑和凸起
    process_contours(pit_mask, diff_pit, "pit");
    process_contours(bump_mask, diff_bump, "bump");
    // 保存标注检测图
    imwrite("result_images/global_detect.jpg", vis_img);
    return event_list;
}
} // anonymous namespace

// 构造函数：加载缺陷检测参数，确保核宽度为奇数
DefectDetector::DefectDetector(const std::string& config_path)
    : cuda_enabled(false),
      defect_thresh_mm(80.0),
      min_long_mm(300.0),
      min_short_mm(100.0),
      margin_x(35),
      margin_y(45),
      bg_kernel_width(301),
      morph_kernel_size(5),
      max_valid_height_mm(2000.0),
      min_valid_height_mm(0.0),
      draw_filtered(false) {
    // 确保结果图输出目录存在
    if (!fs::exists("result_images")) fs::create_directory("result_images");
    try {
        std::ifstream ifs(config_path);
        if (ifs.good()) {
            json cfg = json::parse(ifs, nullptr, false);
            if (!cfg.is_discarded()) {
                // 从 system 段读取有效高度范围
                if (cfg.contains("system")) {
                    const json& s = cfg["system"];
                    max_valid_height_mm = s.value("max_valid_height_mm", max_valid_height_mm);
                    min_valid_height_mm = s.value("min_valid_height_mm", min_valid_height_mm);
                }
                // 从 defect 段读取检测参数
                if (cfg.contains("defect")) {
                    const json& d = cfg["defect"];
                    defect_thresh_mm = d.value("threshold_mm", defect_thresh_mm);
                    min_long_mm = d.value("min_long_mm", min_long_mm);
                    min_short_mm = d.value("min_short_mm", min_short_mm);
                    margin_x = d.value("margin_x_px", margin_x);
                    margin_y = d.value("margin_y_px", margin_y);
                    bg_kernel_width = d.value("bg_kernel_width", bg_kernel_width);
                    morph_kernel_size = d.value("morph_kernel_size", morph_kernel_size);
                    draw_filtered = d.value("draw_filtered", draw_filtered);
                }
            }
        }
    } catch (...) {
    }
    // boxFilter 和形态学核必须为奇数
    if (bg_kernel_width < 3) bg_kernel_width = 3;
    if (bg_kernel_width % 2 == 0) bg_kernel_width += 1;
    if (morph_kernel_size < 1) morph_kernel_size = 1;
    if (morph_kernel_size % 2 == 0) morph_kernel_size += 1;
    // 检测 CUDA 可用性
#if defined(HAVE_OPENCV_CUDAIMGPROC) && defined(HAVE_OPENCV_CUDAFILTERS) && defined(HAVE_OPENCV_CUDAARITHM)
    if (cv::cuda::getCudaEnabledDeviceCount() > 0) {
        cv::cuda::setDevice(0);
        cuda_enabled = true;
    }
#endif
}

// 缺陷检测入口：根据 CUDA 可用性自动选择计算路径
json DefectDetector::detect_global(const Mat& dem_map) {
    if (dem_map.empty()) return json::array();
#if defined(HAVE_OPENCV_CUDAIMGPROC) && defined(HAVE_OPENCV_CUDAFILTERS) && defined(HAVE_OPENCV_CUDAARITHM)
    if (cuda_enabled) return detect_global_cuda(dem_map);
#endif
    return detect_global_cpu(dem_map);
}

/**
 * CPU 检测路径
 *
 * 算法步骤：
 *   1. DEM（米）转毫米高程图
 *   2. 超出有效范围的像素替换为安全背景高度（避免干扰检测）
 *   3. 水平 boxFilter 估计局部背景
 *   4. 计算凹坑偏差（背景-高度）和凸起偏差（高度-背景）
 *   5. 阈值分割 + 安全边距掩码
 *   6. 形态学开运算去噪
 *   7. 轮廓提取，按尺寸和深度过滤
 */
json DefectDetector::detect_global_cpu(const Mat& dem_map) {
    // DEM 转毫米
    Mat height_mm;
    dem_map.convertTo(height_mm, CV_32F, 1000.0);

    // 计算有效区域均值作为安全背景高度
    Mat valid_mask = (height_mm < max_valid_height_mm) & (height_mm > min_valid_height_mm);
    Scalar mean_val = mean(height_mm, valid_mask);
    double safe_bg_h = mean_val[0];
    if (std::isnan(safe_bg_h) || safe_bg_h < 50) safe_bg_h = 1000.0;

    // 超出有效范围的像素替换为安全背景高度
    height_mm.setTo(safe_bg_h, height_mm >= max_valid_height_mm);

    // 水平方向 boxFilter 估计背景（核宽度 bg_kernel_width，高度 1）
    Mat bg;
    boxFilter(height_mm, bg, -1, Size(bg_kernel_width, 1));

    // 计算偏差：凹坑 = 背景 - 高度，凸起 = 高度 - 背景
    Mat diff_pit = bg - height_mm;
    Mat diff_bump = height_mm - bg;

    // 安全边距掩码：排除图像边缘区域的干扰
    Mat safe_mask = Mat::zeros(height_mm.size(), CV_8UC1);
    if (height_mm.cols > 2 * margin_x && height_mm.rows > 2 * margin_y) {
        safe_mask(Rect(margin_x, margin_y, height_mm.cols - 2 * margin_x, height_mm.rows - 2 * margin_y)).setTo(255);
    }

    // 阈值分割并与安全掩码取交集
    Mat pit_mask = (diff_pit > defect_thresh_mm) & safe_mask;
    Mat bump_mask = (diff_bump > defect_thresh_mm) & safe_mask;

    // 形态学开运算去除小噪声
    Mat element = getStructuringElement(MORPH_RECT, Size(morph_kernel_size, morph_kernel_size));
    morphologyEx(pit_mask, pit_mask, MORPH_OPEN, element);
    morphologyEx(bump_mask, bump_mask, MORPH_OPEN, element);

    return build_event_list(height_mm, pit_mask, bump_mask, diff_pit, diff_bump, defect_thresh_mm, min_long_mm, min_short_mm, draw_filtered);
}

#if defined(HAVE_OPENCV_CUDAIMGPROC) && defined(HAVE_OPENCV_CUDAFILTERS) && defined(HAVE_OPENCV_CUDAARITHM)
/**
 * CUDA 加速检测路径，算法逻辑与 CPU 版本一致，
 * 在 GPU 上执行 boxFilter、阈值分割和形态学运算以提升性能
 */
json DefectDetector::detect_global_cuda(const Mat& dem_map) {
    // 上传 DEM 到 GPU 并转为毫米
    cv::cuda::GpuMat g_dem;
    g_dem.upload(dem_map);
    cv::cuda::GpuMat g_height_mm;
    g_dem.convertTo(g_height_mm, CV_32F, 1000.0);

    // 在 GPU 上构建有效范围掩码
    cv::cuda::GpuMat g_lt_2000;
    cv::cuda::GpuMat g_gt_0;
    cv::cuda::GpuMat g_valid_mask;
    cv::cuda::compare(g_height_mm, Scalar(max_valid_height_mm), g_lt_2000, CMP_LT);
    cv::cuda::compare(g_height_mm, Scalar(min_valid_height_mm), g_gt_0, CMP_GT);
    cv::cuda::bitwise_and(g_lt_2000, g_gt_0, g_valid_mask);

    // 下载到 CPU 计算安全背景高度（GPU 上无直接 mean 接口）
    Mat height_mm;
    Mat valid_mask;
    g_height_mm.download(height_mm);
    g_valid_mask.download(valid_mask);

    double safe_bg_h = mean(height_mm, valid_mask)[0];
    if (std::isnan(safe_bg_h) || safe_bg_h < 50) safe_bg_h = 1000.0;
    height_mm.setTo(safe_bg_h, height_mm >= max_valid_height_mm);
    g_height_mm.upload(height_mm);

    // GPU boxFilter 背景估计
    Ptr<cv::cuda::Filter> box_filter = cv::cuda::createBoxFilter(CV_32F, CV_32F, Size(bg_kernel_width, 1));
    cv::cuda::GpuMat g_bg;
    box_filter->apply(g_height_mm, g_bg);

    // GPU 计算偏差
    cv::cuda::GpuMat g_diff_pit;
    cv::cuda::GpuMat g_diff_bump;
    cv::cuda::subtract(g_bg, g_height_mm, g_diff_pit);
    cv::cuda::subtract(g_height_mm, g_bg, g_diff_bump);

    // 安全边距掩码（在 CPU 构建后上传）
    Mat safe_mask = Mat::zeros(dem_map.size(), CV_8UC1);
    if (dem_map.cols > 2 * margin_x && dem_map.rows > 2 * margin_y) {
        safe_mask(Rect(margin_x, margin_y, dem_map.cols - 2 * margin_x, dem_map.rows - 2 * margin_y)).setTo(255);
    }
    cv::cuda::GpuMat g_safe_mask;
    g_safe_mask.upload(safe_mask);

    // GPU 阈值分割
    cv::cuda::GpuMat g_pit_mask_raw;
    cv::cuda::GpuMat g_bump_mask_raw;
    cv::cuda::compare(g_diff_pit, Scalar(defect_thresh_mm), g_pit_mask_raw, CMP_GT);
    cv::cuda::compare(g_diff_bump, Scalar(defect_thresh_mm), g_bump_mask_raw, CMP_GT);

    // 与安全掩码取交集
    cv::cuda::GpuMat g_pit_mask;
    cv::cuda::GpuMat g_bump_mask;
    cv::cuda::bitwise_and(g_pit_mask_raw, g_safe_mask, g_pit_mask);
    cv::cuda::bitwise_and(g_bump_mask_raw, g_safe_mask, g_bump_mask);

    // GPU 形态学开运算
    Mat element = getStructuringElement(MORPH_RECT, Size(morph_kernel_size, morph_kernel_size));
    Ptr<cv::cuda::Filter> morph_filter = cv::cuda::createMorphologyFilter(MORPH_OPEN, CV_8UC1, element);
    morph_filter->apply(g_pit_mask, g_pit_mask);
    morph_filter->apply(g_bump_mask, g_bump_mask);

    // 下载结果到 CPU 进行轮廓分析
    Mat pit_mask;
    Mat bump_mask;
    Mat diff_pit;
    Mat diff_bump;
    g_pit_mask.download(pit_mask);
    g_bump_mask.download(bump_mask);
    g_diff_pit.download(diff_pit);
    g_diff_bump.download(diff_bump);

    return build_event_list(height_mm, pit_mask, bump_mask, diff_pit, diff_bump, defect_thresh_mm, min_long_mm, min_short_mm, draw_filtered);
}
#endif
