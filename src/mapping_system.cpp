/**
 * @file mapping_system.cpp
 * @brief MappingSystem 实现 -- 线扫描拼接、PLY 导出、体积计算
 */
#include "mapping_system.hpp"
#include <filesystem>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <algorithm>
#include "json.hpp"
#if defined(HAVE_OPENCV_CUDAIMGPROC) && defined(HAVE_OPENCV_CUDAARITHM)
#include <opencv2/core/cuda.hpp>
#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudawarping.hpp>
#endif

using namespace std;
using namespace cv;
using json = nlohmann::json;

// 构造函数：加载配置并初始化 DEM
MappingSystem::MappingSystem(const string& config_path) {
    // 默认配置值
    config = {2700.0, 0.132, 5.0, 100, 150.0, 200, 6000, 0.02, 2};
    try {
        std::ifstream ifs(config_path);
        if (ifs.good()) {
            json cfg = json::parse(ifs, nullptr, false);
            if (!cfg.is_discarded()) {
                // 从 system 段读取传感器和物理参数
                if (cfg.contains("system")) {
                    const json& s = cfg["system"];
                    config.install_height = s.value("install_height_mm", config.install_height);
                    config.speed = s.value("belt_speed_m_s", config.speed);
                    config.fps = s.value("fps", config.fps);
                    config.px_per_m = s.value("px_per_m", config.px_per_m);
                    config.rho = s.value("material_rho", config.rho);
                    config.min_object_height_m = s.value("min_object_height_m", config.min_object_height_m);
                }
                // 从 mapping 段读取 DEM 尺寸和 PLY 降采样参数
                if (cfg.contains("mapping")) {
                    const json& m = cfg["mapping"];
                    config.dem_rows = m.value("dem_rows", config.dem_rows);
                    config.dem_cols = m.value("dem_cols", config.dem_cols);
                    config.ply_stride = m.value("ply_stride", config.ply_stride);
                }
            }
        }
    } catch (...) {
    }
    // 检测 CUDA 可用性，自动切换 GPU 加速
#if defined(HAVE_OPENCV_CUDAIMGPROC) && defined(HAVE_OPENCV_CUDAARITHM)
    if (cv::cuda::getCudaEnabledDeviceCount() > 0) {
        cv::cuda::setDevice(0);
        cuda_enabled = true;
    }
#endif
    reset();
}

// 重置拼接状态：清空 DEM 矩阵，拼接位置归零
void MappingSystem::reset() {
    max_end_px = 0;
    dem_matrix = Mat::zeros(config.dem_rows, config.dem_cols, CV_32FC1);
}

/**
 * 单帧处理：将一帧深度图拼接到全局 DEM
 *
 * 处理步骤：
 *   1. 顺时针旋转 90 度，使传送带方向对齐 DEM 列方向
 *   2. 根据帧号和物理速度计算该帧在 DEM 中的绝对像素列范围
 *   3. 从旋转图的中心区域提取切片（中心区域畸变最小）
 *   4. 将 16 位深度值转换为物理高度：h = install_height - depth
 *   5. 缩放切片到 DEM 行高，粘贴到 DEM 对应位置
 */
void MappingSystem::process_frame(const Mat& depth_img_16u, int frame_idx) {
    if (depth_img_16u.empty()) return;

    // 顺时针旋转 90 度（640x360 -> 360x640），使高度方向对齐 DEM 行
    Mat r_depth;
    rotate(depth_img_16u, r_depth, ROTATE_90_CLOCKWISE);

    // 根据帧号计算绝对像素位置，避免浮点累积误差
    double exact_px = frame_idx * (config.speed / config.fps) * config.px_per_m;
    int end_px = std::round(exact_px);
    int start_px = this->max_end_px;
    int slice_w_px = end_px - start_px;

    // 切片宽度无效或超出 DEM 范围则跳过
    if (slice_w_px <= 0 || end_px >= dem_matrix.cols) return;

    // 从旋转图中心提取切片（中心区域精度最高）
    int mid_x = r_depth.cols / 2;
    int half_w = slice_w_px / 2;
    int roi_x = std::max(0, mid_x - half_w);
    int roi_w = std::min(slice_w_px, r_depth.cols - roi_x);

    Rect roi(roi_x, 0, roi_w, r_depth.rows);
    Mat depth_slice = r_depth(roi);

    // 安装高度（米），用于深度值到物体高度的转换
    float h_install_m = config.install_height / 1000.0f;
    Mat resized_slice;

    // CUDA 路径：在 GPU 上完成深度转高度、阈值截断、缩放
#if defined(HAVE_OPENCV_CUDAIMGPROC) && defined(HAVE_OPENCV_CUDAARITHM)
    if (cuda_enabled) {
        cv::cuda::GpuMat g_depth_slice;
        g_depth_slice.upload(depth_slice);
        cv::cuda::GpuMat g_height_slice;
        // h = install_height - depth/1000（线性变换：scale=-0.001, offset=h_install_m）
        g_depth_slice.convertTo(g_height_slice, CV_32FC1, -0.001, h_install_m);
        // 负高度截断为 0
        cv::cuda::threshold(g_height_slice, g_height_slice, 0.0, 0.0, THRESH_TOZERO);
        // 超过安装高度的值截断（传感器噪声）
        cv::cuda::GpuMat g_overflow_mask;
        cv::cuda::compare(g_height_slice, Scalar(h_install_m), g_overflow_mask, CMP_GT);
        g_height_slice.setTo(Scalar(h_install_m), g_overflow_mask);
        // 缩放到 DEM 行高
        cv::cuda::GpuMat g_resized_slice;
        cv::cuda::resize(g_height_slice, g_resized_slice, Size(slice_w_px, dem_matrix.rows), 0, 0, INTER_LINEAR);
        g_resized_slice.download(resized_slice);
    } else
#endif
    {
        // CPU 路径
        Mat depth_slice_f;
        depth_slice.convertTo(depth_slice_f, CV_32FC1);
        // 深度转高度：h = install_height(m) - depth(mm)/1000
        Mat height_slice = h_install_m - (depth_slice_f / 1000.0f);
        // 负高度截断为 0
        threshold(height_slice, height_slice, 0.0, 0.0, THRESH_TOZERO);
        // 超过安装高度的值截断
        Mat overflow_mask = height_slice > h_install_m;
        height_slice.setTo(h_install_m, overflow_mask);
        // 双线性插值缩放到 DEM 行高
        resize(height_slice, resized_slice, Size(slice_w_px, dem_matrix.rows), 0, 0, INTER_LINEAR);
    }

    // 粘贴到全局 DEM 的对应列位置
    resized_slice.copyTo(dem_matrix(Rect(start_px, 0, slice_w_px, dem_matrix.rows)));
    this->max_end_px = end_px;
}

/**
 * 批量处理多帧深度图
 * CUDA 可用时使用 GPU 路径逐帧处理（减少 CPU-GPU 上下文切换开销）
 * 否则回退到 CPU 路径逐帧调用 process_frame
 */
void MappingSystem::process_frame_batch(const vector<Mat>& depth_batch, const vector<int>& frame_ids) {
    if (depth_batch.empty() || depth_batch.size() != frame_ids.size()) return;

#if defined(HAVE_OPENCV_CUDAIMGPROC) && defined(HAVE_OPENCV_CUDAARITHM)
    if (cuda_enabled) {
        float h_install_m = config.install_height / 1000.0f;
        for (size_t i = 0; i < depth_batch.size(); ++i) {
            const Mat& depth_img_16u = depth_batch[i];
            int frame_idx = frame_ids[i];
            if (depth_img_16u.empty()) continue;

            Mat r_depth;
            rotate(depth_img_16u, r_depth, ROTATE_90_CLOCKWISE);

            double exact_px = frame_idx * (config.speed / config.fps) * config.px_per_m;
            int end_px = std::round(exact_px);
            int start_px = this->max_end_px;
            int slice_w_px = end_px - start_px;
            if (slice_w_px <= 0 || end_px >= dem_matrix.cols) continue;

            int mid_x = r_depth.cols / 2;
            int half_w = slice_w_px / 2;
            int roi_x = std::max(0, mid_x - half_w);
            int roi_w = std::min(slice_w_px, r_depth.cols - roi_x);
            Rect roi(roi_x, 0, roi_w, r_depth.rows);
            Mat depth_slice = r_depth(roi);

            try {
                cv::cuda::GpuMat g_depth_slice;
                g_depth_slice.upload(depth_slice);
                cv::cuda::GpuMat g_height_slice;
                g_depth_slice.convertTo(g_height_slice, CV_32FC1, -0.001, h_install_m);
                cv::cuda::threshold(g_height_slice, g_height_slice, 0.0, 0.0, THRESH_TOZERO);
                cv::cuda::GpuMat g_overflow_mask;
                cv::cuda::compare(g_height_slice, Scalar(h_install_m), g_overflow_mask, CMP_GT);
                g_height_slice.setTo(Scalar(h_install_m), g_overflow_mask);
                cv::cuda::GpuMat g_resized_slice;
                cv::cuda::resize(g_height_slice, g_resized_slice, Size(slice_w_px, dem_matrix.rows), 0, 0, INTER_LINEAR);
                Mat resized_slice;
                g_resized_slice.download(resized_slice);

                resized_slice.copyTo(dem_matrix(Rect(start_px, 0, slice_w_px, dem_matrix.rows)));
                this->max_end_px = end_px;
            } catch (const cv::Exception& e) {
                std::cerr << "[GPU Error] Frame " << frame_idx << ": " << e.what() << std::endl;
                continue;
            }
        }
        return;
    }
#endif
    // CPU 回退：逐帧调用 process_frame
    for (size_t i = 0; i < depth_batch.size(); ++i) {
        process_frame(depth_batch[i], frame_ids[i]);
    }
}

/**
 * 体积计算：遍历 DEM 有效区域，对每个高于阈值的像素累加 h * cell_area
 * cell_area = (1/px_per_m)^2，即每个像素对应的物理面积（m2）
 */
double MappingSystem::calculate_global_volume(double& out_mass) {
    if (max_end_px == 0) return 0.0;

    double total_vol = 0.0;
    double area_per_px = (1.0 / config.px_per_m) * (1.0 / config.px_per_m);

    // 仅遍历已拼接的有效区域
    Rect valid_roi(0, 0, max_end_px, dem_matrix.rows);
    Mat valid_dem = dem_matrix(valid_roi);

    for (int r = 0; r < valid_dem.rows; r++) {
        for (int c = 0; c < valid_dem.cols; c++) {
            float h = valid_dem.at<float>(r, c);
            if (h > config.min_object_height_m) {
                total_vol += (h * area_per_px);
            }
        }
    }

    out_mass = total_vol * config.rho; // 质量 = 体积 x 密度
    return total_vol;
}

/**
 * PLY 导出：从 DEM 按 ply_stride 降采样，将有效高度点转换为物理坐标写入 ASCII PLY
 * 坐标系：X = 传送带方向（列/px_per_m），Y = 垂直传送带方向（行/px_per_m - 1.0），Z = 高度
 */
string MappingSystem::generate_full_ply(int batch_id, const string& output_dir) {
    if (!std::filesystem::exists(output_dir)) std::filesystem::create_directories(output_dir);
    string file_path = output_dir + "/Global_Stitched_" + to_string(batch_id) + ".ply";
    ofstream f(file_path);

    if (max_end_px == 0) return "";

    // 收集有效点
    std::vector<Point3D> points;

    int stride = std::max(1, config.ply_stride);
    for (int r = 0; r < dem_matrix.rows; r += stride) {
        for (int c = 0; c < max_end_px; c += stride) {
            float h = dem_matrix.at<float>(r, c);
            if (h > config.min_object_height_m) {
                float x_m = (float)c / config.px_per_m;
                float y_m = (float)r / config.px_per_m - 1.0f;
                points.push_back({x_m, y_m, h});
            }
        }
    }

    // 写入 PLY 文件头和点数据
    f << "ply\nformat ascii 1.0\nelement vertex " << points.size()
      << "\nproperty float x\nproperty float y\nproperty float z\nend_header\n";
    for (const auto& pt : points) {
        f << fixed << setprecision(4) << pt.x << " " << pt.y << " " << pt.z << "\n";
    }
    return file_path;
}

// 获取 DEM 有效区域的深拷贝，供缺陷检测使用
Mat MappingSystem::get_dem_map() const {
    if (max_end_px == 0) return dem_matrix;
    return dem_matrix(Rect(0, 0, max_end_px, dem_matrix.rows)).clone();
}

// 获取当前拼接长度（米）
double MappingSystem::get_stitched_length_m() const {
    return (double)max_end_px / config.px_per_m;
}
