#include "vision_interface.hpp"
#include "mapping_system.hpp"
#include "defect_detector.hpp"
#include <iostream>

// 全局映射系统实例
static MappingSystem* g_mapper = nullptr;
// 全局缺陷检测器实例
static DefectDetector* g_detector = nullptr;

void TobaccoVisionSystem::init(const std::string& configPath) {
    if (!g_mapper) g_mapper = new MappingSystem(configPath);
    if (!g_detector) g_detector = new DefectDetector(configPath);
    std::cout << "[Vision] System Initialized (Global Line-Scan + CPU Detector)." << std::endl;
}

void TobaccoVisionSystem::resetSession() {
    if (g_mapper) g_mapper->reset();
}

FlowResult TobaccoVisionSystem::analyzeConveyorFlow(const ImageFrame& frame, double elapsed) {
    return { true, 1.0, elapsed };
}

CarriageResult TobaccoVisionSystem::detectCarriageState(const ImageFrame& rgbFrame, const std::string& sideInfo, double elapsed) {
    return { CarriageState::STOPPED, "Not implemented yet", elapsed };
}

std::string TobaccoVisionSystem::generatePointCloud(const CameraStreamData& streams, int frameId) {
    // 查找相机101的数据，如果没有则使用第一个相机
    auto it = streams.find(101);
    if (it == streams.end() && !streams.empty()) it = streams.begin();
    if (it == streams.end() || it->second.empty()) return "cached";

    // 解码图像
    const std::vector<unsigned char>& raw = it->second.back();
    cv::Mat depth = cv::imdecode(raw, cv::IMREAD_UNCHANGED);

    // 确保是16位深度图
    if (!depth.empty() && depth.type() != CV_16UC1) {
        cv::Mat depth16;
        depth.convertTo(depth16, CV_16UC1);
        return processFrameMat(depth16, frameId);
    }
    return processFrameMat(depth, frameId);
}

std::string TobaccoVisionSystem::processFrameMat(const cv::Mat& depth, int frameId) {
    if (depth.empty() || depth.type() != CV_16UC1) {
        std::cerr << "[Error] Frame " << frameId << " is empty or not 16bit!" << std::endl;
        return "error";
    }
    if (g_mapper) g_mapper->process_frame(depth, frameId);
    return "processed";
}

void TobaccoVisionSystem::processFrameBatch(const std::vector<cv::Mat>& depth_batch, const std::vector<int>& frame_ids) {
    if (g_mapper) g_mapper->process_frame_batch(depth_batch, frame_ids);
}

std::string TobaccoVisionSystem::generateFullPly(int batchId) {
    if (g_mapper) return g_mapper->generate_full_ply(batchId, "output_ply");
    return "";
}

double TobaccoVisionSystem::calculateVolume(const std::string& plyPath, double& outMass) {
    if (g_mapper) return g_mapper->calculate_global_volume(outMass);
    outMass = 0.0;
    return 0.0;
}

double TobaccoVisionSystem::getStitchedLength() {
    if (g_mapper) return g_mapper->get_stitched_length_m();
    return 0.0;
}

// ZMQ发布专用接口
json TobaccoVisionSystem::detectAnomaliesByMat() {
    if (!g_mapper || !g_detector) return json::array();

    cv::Mat dem_map = g_mapper->get_dem_map();
    if (dem_map.empty()) return json::array();

    // 调用全局检测逻辑
    return g_detector->detect_global(dem_map);
}

AnomalyResult TobaccoVisionSystem::detectAnomalies(const std::string& plyPath) {
    json res = detectAnomaliesByMat();
    if (res.size() > 0) {
        double max_sev = 0.0;
        std::string main_type = "MULTIPLE";
        for(auto& item : res) {
            if(std::abs((double)item["dev_mm"]) > std::abs(max_sev)) {
                max_sev = item["dev_mm"];
                main_type = item["type"];
            }
        }
        return {true, main_type, max_sev, res.dump(4)};
    }
    return {false, "NORMAL", 0.0, "[]"};
}
