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
// 构建缺陷事件列表
json build_event_list(const Mat& height_mm, Mat pit_mask, Mat bump_mask, const Mat& diff_pit, const Mat& diff_bump, double defect_thresh_mm, double min_long_mm, double min_short_mm, bool draw_filtered) {
    json event_list = json::array();
    // 创建可视化图像
    Mat vis_img;
    normalize(height_mm, vis_img, 0, 255, NORM_MINMAX, CV_8UC1);
    cvtColor(vis_img, vis_img, COLOR_GRAY2BGR);

    // 处理轮廓的lambda函数
    auto process_contours = [&](Mat& mask, const Mat& diff, const string& type) {
        vector<vector<Point>> contours;
        findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        for (size_t i = 0; i < contours.size(); i++) {
            Rect r = boundingRect(contours[i]);
            double length_mm = r.width * 10.0;  // 像素转毫米
            double width_mm = r.height * 10.0;

            // 获取最大偏差值
            double max_v = 0;
            minMaxLoc(diff(r), nullptr, &max_v, nullptr, nullptr, mask(r));

            // 检查深度和尺寸阈值
            bool check_depth = (max_v >= defect_thresh_mm);
            bool check_size = (std::max(length_mm, width_mm) >= min_long_mm) && (std::min(length_mm, width_mm) >= min_short_mm);

            if (check_depth && check_size) {
                // 构造缺陷事件
                json evt;
                evt["type"] = type;
                evt["dev_mm"] = (type == "pit" ? -max_v : max_v);
                evt["width_mm"] = width_mm;
                evt["length_mm"] = length_mm;
                evt["center_x_m"] = (r.x + r.width / 2.0) * 0.01;
                evt["center_y_m"] = (r.y + r.height / 2.0) * 0.01 - 1.0;
                event_list.push_back(evt);

                // 绘制缺陷框
                Scalar color = (type == "pit") ? Scalar(0, 0, 255) : Scalar(0, 255, 255);
                rectangle(vis_img, r, color, 3);
                string label = type + " " + to_string((int)max_v) + "mm";
                putText(vis_img, label, Point(r.x, r.y - 10), FONT_HERSHEY_SIMPLEX, 0.8, color, 2);
            } else if (max_v >= defect_thresh_mm) {
                // 被尺寸过滤的缺陷
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

    process_contours(pit_mask, diff_pit, "pit");
    process_contours(bump_mask, diff_bump, "bump");
    imwrite("result_images/global_detect.jpg", vis_img);
    return event_list;
}
}

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
    // 创建结果图像目录
    if (!fs::exists("result_images")) fs::create_directory("result_images");
    // 加载配置文件
    try {
        std::ifstream ifs(config_path);
        if (ifs.good()) {
            json cfg = json::parse(ifs, nullptr, false);
            if (!cfg.is_discarded()) {
                if (cfg.contains("system")) {
                    const json& s = cfg["system"];
                    max_valid_height_mm = s.value("max_valid_height_mm", max_valid_height_mm);
                    min_valid_height_mm = s.value("min_valid_height_mm", min_valid_height_mm);
                }
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
    if (bg_kernel_width < 3) bg_kernel_width = 3;
    if (bg_kernel_width % 2 == 0) bg_kernel_width += 1;
    if (morph_kernel_size < 1) morph_kernel_size = 1;
    if (morph_kernel_size % 2 == 0) morph_kernel_size += 1;
#if defined(HAVE_OPENCV_CUDAIMGPROC) && defined(HAVE_OPENCV_CUDAFILTERS) && defined(HAVE_OPENCV_CUDAARITHM)
    if (cv::cuda::getCudaEnabledDeviceCount() > 0) {
        cv::cuda::setDevice(0);
        cuda_enabled = true;
    }
#endif
}

json DefectDetector::detect_global(const Mat& dem_map) {
    if (dem_map.empty()) return json::array();
#if defined(HAVE_OPENCV_CUDAIMGPROC) && defined(HAVE_OPENCV_CUDAFILTERS) && defined(HAVE_OPENCV_CUDAARITHM)
    if (cuda_enabled) return detect_global_cuda(dem_map);
#endif
    return detect_global_cpu(dem_map);
}

json DefectDetector::detect_global_cpu(const Mat& dem_map) {
    Mat height_mm;
    dem_map.convertTo(height_mm, CV_32F, 1000.0);

    Mat valid_mask = (height_mm < max_valid_height_mm) & (height_mm > min_valid_height_mm);
    Scalar mean_val = mean(height_mm, valid_mask);
    double safe_bg_h = mean_val[0];
    if (std::isnan(safe_bg_h) || safe_bg_h < 50) safe_bg_h = 1000.0;

    height_mm.setTo(safe_bg_h, height_mm >= max_valid_height_mm);

    Mat bg;
    boxFilter(height_mm, bg, -1, Size(bg_kernel_width, 1));

    Mat diff_pit = bg - height_mm;
    Mat diff_bump = height_mm - bg;

    Mat safe_mask = Mat::zeros(height_mm.size(), CV_8UC1);
    if (height_mm.cols > 2 * margin_x && height_mm.rows > 2 * margin_y) {
        safe_mask(Rect(margin_x, margin_y, height_mm.cols - 2 * margin_x, height_mm.rows - 2 * margin_y)).setTo(255);
    }

    Mat pit_mask = (diff_pit > defect_thresh_mm) & safe_mask;
    Mat bump_mask = (diff_bump > defect_thresh_mm) & safe_mask;

    Mat element = getStructuringElement(MORPH_RECT, Size(morph_kernel_size, morph_kernel_size));
    morphologyEx(pit_mask, pit_mask, MORPH_OPEN, element);
    morphologyEx(bump_mask, bump_mask, MORPH_OPEN, element);

    return build_event_list(height_mm, pit_mask, bump_mask, diff_pit, diff_bump, defect_thresh_mm, min_long_mm, min_short_mm, draw_filtered);
}

#if defined(HAVE_OPENCV_CUDAIMGPROC) && defined(HAVE_OPENCV_CUDAFILTERS) && defined(HAVE_OPENCV_CUDAARITHM)
json DefectDetector::detect_global_cuda(const Mat& dem_map) {
    cv::cuda::GpuMat g_dem;
    g_dem.upload(dem_map);

    cv::cuda::GpuMat g_height_mm;
    g_dem.convertTo(g_height_mm, CV_32F, 1000.0);

    cv::cuda::GpuMat g_lt_2000;
    cv::cuda::GpuMat g_gt_0;
    cv::cuda::GpuMat g_valid_mask;
    cv::cuda::compare(g_height_mm, Scalar(max_valid_height_mm), g_lt_2000, CMP_LT);
    cv::cuda::compare(g_height_mm, Scalar(min_valid_height_mm), g_gt_0, CMP_GT);
    cv::cuda::bitwise_and(g_lt_2000, g_gt_0, g_valid_mask);

    Mat height_mm;
    Mat valid_mask;
    g_height_mm.download(height_mm);
    g_valid_mask.download(valid_mask);

    double safe_bg_h = mean(height_mm, valid_mask)[0];
    if (std::isnan(safe_bg_h) || safe_bg_h < 50) safe_bg_h = 1000.0;
    height_mm.setTo(safe_bg_h, height_mm >= max_valid_height_mm);
    g_height_mm.upload(height_mm);

    Ptr<cv::cuda::Filter> box_filter = cv::cuda::createBoxFilter(CV_32F, CV_32F, Size(bg_kernel_width, 1));
    cv::cuda::GpuMat g_bg;
    box_filter->apply(g_height_mm, g_bg);

    cv::cuda::GpuMat g_diff_pit;
    cv::cuda::GpuMat g_diff_bump;
    cv::cuda::subtract(g_bg, g_height_mm, g_diff_pit);
    cv::cuda::subtract(g_height_mm, g_bg, g_diff_bump);

    Mat safe_mask = Mat::zeros(dem_map.size(), CV_8UC1);
    if (dem_map.cols > 2 * margin_x && dem_map.rows > 2 * margin_y) {
        safe_mask(Rect(margin_x, margin_y, dem_map.cols - 2 * margin_x, dem_map.rows - 2 * margin_y)).setTo(255);
    }
    cv::cuda::GpuMat g_safe_mask;
    g_safe_mask.upload(safe_mask);

    cv::cuda::GpuMat g_pit_mask_raw;
    cv::cuda::GpuMat g_bump_mask_raw;
    cv::cuda::compare(g_diff_pit, Scalar(defect_thresh_mm), g_pit_mask_raw, CMP_GT);
    cv::cuda::compare(g_diff_bump, Scalar(defect_thresh_mm), g_bump_mask_raw, CMP_GT);

    cv::cuda::GpuMat g_pit_mask;
    cv::cuda::GpuMat g_bump_mask;
    cv::cuda::bitwise_and(g_pit_mask_raw, g_safe_mask, g_pit_mask);
    cv::cuda::bitwise_and(g_bump_mask_raw, g_safe_mask, g_bump_mask);

    Mat element = getStructuringElement(MORPH_RECT, Size(morph_kernel_size, morph_kernel_size));
    Ptr<cv::cuda::Filter> morph_filter = cv::cuda::createMorphologyFilter(MORPH_OPEN, CV_8UC1, element);
    morph_filter->apply(g_pit_mask, g_pit_mask);
    morph_filter->apply(g_bump_mask, g_bump_mask);

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
