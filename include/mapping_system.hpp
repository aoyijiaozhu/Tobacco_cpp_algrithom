#pragma once
#include <opencv2/opencv.hpp>
#include <vector>
#include <string>

struct SystemConfig {
    double install_height;
    double speed;
    double fps;
    int px_per_m;
    double rho;
    int dem_rows;
    int dem_cols;
    double min_object_height_m;
    int ply_stride;
};

struct Point3D {
    float x, y, z;
};

class MappingSystem {
public:
    MappingSystem(const std::string& config_path);
    void reset();
    
    // 纯物理线扫描拼接 (Python 逻辑复刻)
    void process_frame(const cv::Mat& depth_img_16u, int frame_idx);
    
    // 从拼接好的高程图生成无断层点云 (test_stitching_only.py 逻辑)
    std::string generate_full_ply(int batch_id, const std::string& output_dir = "output_ply");
    
    // 计算全局积分体积和质量 [新增]
    double calculate_global_volume(double& out_mass);

    cv::Mat get_dem_map() const;
    double get_stitched_length_m() const;

private:
    SystemConfig config;
    bool cuda_enabled = false;
    cv::Mat dem_matrix;    // 全局物理高程图 (CV_32FC1)，直接存高度(米)
    int max_end_px = 0;    // 当前累积拼接到的像素列数
};
