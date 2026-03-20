/**
 * @file analyze_stream.cpp
 * @brief analyzeStream() 实现 -- 编排完整的深度帧分析流水线
 */
#include "AnalyzeStream.h"
#include "../include/mapping_system.hpp"
#include "../include/defect_detector.hpp"
#include <fstream>
#include <iostream>
#include <filesystem>

namespace fs = std::filesystem;

/**
 * 核心分析流水线实现
 *
 * 处理流程：
 *   1. 从 config.json 读取相机帧尺寸，计算每帧期望字节数
 *   2. 遍历每个相机，创建独立的 MappingSystem 和 DefectDetector
 *   3. 校验帧字节大小，将有效帧转换为 cv::Mat
 *   4. 批量拼接到 DEM -> 导出 PLY -> 计算体积 -> 缺陷检测
 *   5. 重命名检测图（加入相机 ID 和批次号避免覆盖）
 *   6. 汇总所有结果为 JSON 返回
 */
std::string analyzeStream(const CameraStreamData& streams, int batchId) {
    const std::string config_path = "config.json";

    // 从 config.json 读取相机帧尺寸，用于校验输入帧
    int frame_width = 640;
    int frame_height = 480;
    {
        std::ifstream ifs(config_path);
        if (ifs.is_open()) {
            json cfg = json::parse(ifs);
            if (cfg.contains("camera")) {
                auto& cam = cfg["camera"];
                if (cam.contains("frame_width"))  frame_width  = cam["frame_width"].get<int>();
                if (cam.contains("frame_height")) frame_height = cam["frame_height"].get<int>();
            }
        }
    }
    // 16 位深度图每像素 2 字节
    const size_t expected_size = static_cast<size_t>(frame_width) * frame_height * 2;

    // 构建返回的 JSON 结构
    json result;
    result["batch_id"] = batchId;
    result["camera_count"] = static_cast<int>(streams.size());
    result["cameras"] = json::array();

    if (streams.empty()) {
        return result.dump();
    }

    // 多相机时 PLY 输出到各自子目录
    bool multi_camera = (streams.size() > 1);

    for (const auto& [camId, frames] : streams) {
        // 每个相机创建独立的拼接系统和检测器，互不干扰
        MappingSystem mapper(config_path);
        DefectDetector detector(config_path);

        std::vector<cv::Mat> valid_mats;
        std::vector<int> valid_ids;

        // 帧校验：检查字节大小，将有效帧转换为 CV_16UC1 的 Mat
        for (size_t i = 0; i < frames.size(); ++i) {
            if (frames[i].size() == expected_size) {
                cv::Mat depth(frame_height, frame_width, CV_16UC1,
                              const_cast<unsigned char*>(frames[i].data()));
                valid_mats.push_back(depth.clone());
                valid_ids.push_back(static_cast<int>(i) + 1);  // 帧序号从 1 开始
            } else {
                std::cerr << "[analyzeStream] cam " << camId
                          << " frame " << i << " size mismatch: "
                          << frames[i].size() << " vs expected " << expected_size
                          << ", skipped" << std::endl;
            }
        }

        // 构建该相机的结果
        json cam_result;
        cam_result["camera_id"] = camId;
        cam_result["total_frames"] = static_cast<int>(frames.size());
        cam_result["valid_frames"] = static_cast<int>(valid_mats.size());

        // 无有效帧时返回空结果
        if (valid_mats.empty()) {
            cam_result["ply_path"] = "";
            cam_result["volume_m3"] = 0.0;
            cam_result["mass_kg"] = 0.0;
            cam_result["defects"] = json::array();
            cam_result["has_anomaly"] = false;
            cam_result["anomaly_image"] = "";
            result["cameras"].push_back(cam_result);
            continue;
        }

        // 批量拼接所有有效帧到 DEM
        mapper.process_frame_batch(valid_mats, valid_ids);

        // 导出 PLY 点云文件
        std::string output_dir = multi_camera
            ? "output_ply/cam_" + std::to_string(camId)
            : "output_ply";
        fs::create_directories(output_dir);
        std::string ply_path = mapper.generate_full_ply(batchId, output_dir);

        // 计算体积和质量
        double mass = 0.0;
        double volume = mapper.calculate_global_volume(mass);

        // 在 DEM 上执行缺陷检测
        cv::Mat dem = mapper.get_dem_map();
        json defects = detector.detect_global(dem);

        // 重命名检测图：加入相机 ID 和批次号，避免多相机时覆盖
        std::string anomaly_image;
        std::string src_img = "result_images/global_detect.jpg";
        if (fs::exists(src_img)) {
            std::string dst_img = "result_images/cam_" + std::to_string(camId)
                                + "_batch_" + std::to_string(batchId) + "_detect.jpg";
            fs::create_directories("result_images");
            fs::rename(src_img, dst_img);
            anomaly_image = dst_img;
        }

        // 填充该相机的结果字段
        cam_result["ply_path"] = ply_path;
        cam_result["volume_m3"] = volume;
        cam_result["mass_kg"] = mass;
        cam_result["defects"] = defects;
        cam_result["has_anomaly"] = !defects.empty();
        cam_result["anomaly_image"] = anomaly_image;
        result["cameras"].push_back(cam_result);
    }

    return result.dump();
}
