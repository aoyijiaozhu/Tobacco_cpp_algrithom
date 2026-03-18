#include <iostream>
#include <string>
#include <vector>
#include <set>
#include <filesystem>
#include <fstream>
#include <algorithm>
#include <cctype>
#include <thread>
#include <chrono>
#include <opencv2/opencv.hpp>
#include "vision_interface.hpp"
#include "json.hpp"
#include "batch_loader.hpp"
#include "zmq_publisher.hpp"
#include "ply_processor.hpp" 

namespace fs = std::filesystem;
using json = nlohmann::json;

/**
 * @brief 自然排序比较函数
 * @details 按文件名长度和字典序排序，确保帧序正确
 */
bool natural_sort(const std::string& a, const std::string& b) {
    std::string fa = fs::path(a).filename().string();
    std::string fb = fs::path(b).filename().string();
    if (fa.length() != fb.length()) return fa.length() < fb.length();
    return fa < fb;
}

/**
 * @brief 读取文件到缓冲区
 * @param path 文件路径
 * @return std::vector<unsigned char> 文件内容
 */
std::vector<unsigned char> readFileToBuffer(const std::string& path) {
    std::ifstream file(path, std::ios::binary);
    if (!file) return {};
    return std::vector<unsigned char>((std::istreambuf_iterator<char>(file)),
                                      std::istreambuf_iterator<char>());
}

/**
 * @brief 运行时配置结构
 */
struct RuntimeConfig {
    std::string dataset_dir = "";              // 数据集目录
    std::string zmq_bind = "tcp://*:5555";     // ZMQ绑定地址
    bool watch_mode = true;                    // 监控模式 (true: 持续监控, false: 单次处理)
    int scan_interval_ms = 500;                // 扫描间隔 (毫秒)
    int publish_every_frames = 20;             // 每N帧发布一次结果
    int camera_id = 101;                       // 相机ID
    int batch_id_start = 20240101;             // 批次ID起始值
    int batch_size = 8;                        // 批处理大小
    bool use_batch_loader = true;              // 是否使用批量加载器
};

/**
 * @brief 加载运行时配置
 * @param config_path 配置文件路径
 * @return RuntimeConfig 运行时配置对象
 */
RuntimeConfig loadRuntimeConfig(const std::string& config_path) {
    RuntimeConfig cfg;
    std::ifstream ifs(config_path);
    if (!ifs.good()) return cfg;
    json root = json::parse(ifs, nullptr, false);
    if (root.is_discarded()) return cfg;
    cfg.dataset_dir = root.value("dataset_path", cfg.dataset_dir);
    if (root.contains("zmq")) {
        const json& z = root["zmq"];
        cfg.zmq_bind = z.value("bind", cfg.zmq_bind);
    }
    if (root.contains("runtime")) {
        const json& r = root["runtime"];
        cfg.dataset_dir = r.value("dataset_dir", cfg.dataset_dir);
        cfg.watch_mode = r.value("watch_mode", cfg.watch_mode);
        cfg.scan_interval_ms = r.value("scan_interval_ms", cfg.scan_interval_ms);
        cfg.publish_every_frames = r.value("publish_every_frames", cfg.publish_every_frames);
        cfg.camera_id = r.value("camera_id", cfg.camera_id);
        cfg.batch_id_start = r.value("batch_id_start", cfg.batch_id_start);
        cfg.batch_size = r.value("batch_size", cfg.batch_size);
        cfg.use_batch_loader = r.value("use_batch_loader", cfg.use_batch_loader);
    }
    if (cfg.publish_every_frames <= 0) cfg.publish_every_frames = 1;
    if (cfg.scan_interval_ms < 50) cfg.scan_interval_ms = 50;
    if (cfg.batch_size <= 0) cfg.batch_size = 8;
    return cfg;
}

/**
 * @brief 解析数据集目录
 * @param from_config 配置文件中的目录路径
 * @return std::string 实际存在的目录路径
 * @details 按优先级查找可用的数据集目录
 */
std::string resolveDatasetDir(const std::string& from_config) {
    std::vector<std::string> candidates;
    if (!from_config.empty()) candidates.push_back(from_config);
    candidates.push_back("test_data/run_01");
    candidates.push_back("dataset/depth");
    candidates.push_back("dataset");
    for (const auto& dir : candidates) {
        if (fs::exists(dir) && fs::is_directory(dir)) return dir;
    }
    return "";
}

/**
 * @brief 列出目录中的图像文件
 * @param dataset_dir 数据集目录
 * @return std::vector<std::string> 排序后的图像文件路径列表
 */
std::vector<std::string> listImageFiles(const std::string& dataset_dir) {
    std::vector<std::string> image_files;
    for (const auto& entry : fs::directory_iterator(dataset_dir)) {
        if (!entry.is_regular_file()) continue;
        std::string ext = entry.path().extension().string();
        std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
        if (ext == ".png" || ext == ".jpg") image_files.push_back(entry.path().string());
    }
    std::sort(image_files.begin(), image_files.end(), natural_sort);
    return image_files;
}

/**
 * @brief 主函数 - 烟草视觉检测系统入口
 * @param argc 参数数量
 * @param argv 参数数组 (argv[1]: 配置文件路径)
 * @return int 退出码
 * @details 初始化系统，启动批量加载器，循环处理图像并发布检测结果
 */
int main(int argc, char** argv) {
    std::string config_path = "config.json";
    if (argc > 1) config_path = argv[1];

    std::cout << ">>> Start: Backend ZMQ Global Line-Scan System <<<" << std::endl;

    RuntimeConfig runtime_cfg = loadRuntimeConfig(config_path);
    TobaccoVisionSystem::init(config_path);

    ZmqPublisher zmq_pub;
    zmq_pub.init(runtime_cfg.zmq_bind);
    std::cout << "[ZMQ] Publisher bound to " << runtime_cfg.zmq_bind << std::endl;

    PlyProcessor ply_processor(&zmq_pub);
    ply_processor.start();
    std::cout << "[PlyProcessor] Started" << std::endl;

    std::string dataset_dir = resolveDatasetDir(runtime_cfg.dataset_dir);
    if (dataset_dir.empty()) {
        std::cerr << "[Error] Dataset directory not found." << std::endl;
        return 1;
    }
    std::cout << "[Data] Using dataset dir: " << dataset_dir << std::endl;
    std::cout << "[Mode] " << (runtime_cfg.watch_mode ? "watch" : "oneshot")
              << " | BatchLoader: " << (runtime_cfg.use_batch_loader ? "enabled" : "disabled")
              << " | BatchSize: " << runtime_cfg.batch_size << std::endl;

    int frameId = 0;
    int last_publish_frame = 0;
    int batch_id = runtime_cfg.batch_id_start;
    int batch_seq = 0;
    std::uint64_t total_read_bytes = 0;
    auto service_start = std::chrono::steady_clock::now();
    auto last_heartbeat = service_start;
    auto batch_start = service_start;

    BatchLoader* loader = nullptr;
    if (runtime_cfg.use_batch_loader) {
        loader = new BatchLoader(dataset_dir, runtime_cfg.batch_size);
        loader->start();
        std::cout << "[BatchLoader] Started with " << runtime_cfg.batch_size << " images per batch" << std::endl;
    }

    std::set<std::string> processed_files;

    /**
     * @brief 发布快照 - 生成PLY、检测缺陷并发布结果
     * @details 执行全局缺陷检测，计算体积质量，通过ZMQ发布
     */
    auto publish_snapshot = [&]() {
        double length_m = TobaccoVisionSystem::getStitchedLength();
        double mass_kg = 0.0;
        double total_vol = TobaccoVisionSystem::calculateVolume("", mass_kg);
        std::string finalPlyPath = TobaccoVisionSystem::generateFullPly(batch_id++);
        std::string detectImagePath = "result_images/global_detect.jpg";

        // 提交PLY文件进行异步处理
        ply_processor.submit(finalPlyPath);

        json defects = TobaccoVisionSystem::detectAnomaliesByMat();
        if (defects.size() > 0) {
            json defects_msg;
            defects_msg["ply_path"] = finalPlyPath;
            defects_msg["image_path"] = detectImagePath;
            defects_msg["events"] = defects;
            zmq_pub.publish("DEFECTS", defects_msg.dump());
            std::cout << "[ZMQ] ALARM! Published " << defects.size() << " defects to topic 'DEFECTS'." << std::endl;
        } else {
            std::cout << "[Global Detect] All Clear. No defects found." << std::endl;
        }

        // 汇总信息（也携带路径）
        json summary;
        summary["total_frames"] = frameId;
        summary["stitched_length_m"] = length_m;
        summary["total_volume_m3"] = total_vol;
        summary["mass_kg"] = mass_kg;
        summary["ply_path"] = finalPlyPath;
        summary["image_path"] = detectImagePath;
        zmq_pub.publish("SUMMARY", summary.dump());

        // 在终端打印本批次的体积、长度和路径
        std::cout << "[Result] length_m=" << length_m
                  << " volume_m3=" << total_vol
                  << " mass_kg=" << mass_kg << std::endl;
        std::cout << "[Result] ply_path=" << finalPlyPath
                  << " image_path=" << detectImagePath << std::endl;
        std::cout << "[ZMQ] SUMMARY published." << std::endl;
        last_publish_frame = frameId;
    };

    while (true) {
        int new_frames = 0;

        if (runtime_cfg.use_batch_loader && loader) {
            BatchData batch;
            if (loader->get_batch(batch, runtime_cfg.scan_interval_ms)) {
                auto batch_read_start = std::chrono::steady_clock::now();
                int batch_size = batch.images.size();

                std::vector<int> corrected_frame_ids;
                for (size_t i = 0; i < batch_size; ++i) {
                    corrected_frame_ids.push_back(frameId + i + 1);
                }

                TobaccoVisionSystem::processFrameBatch(batch.images, corrected_frame_ids);
                frameId += batch_size;
                new_frames += batch_size;

                auto batch_read_end = std::chrono::steady_clock::now();
                auto batch_process_ms = std::chrono::duration_cast<std::chrono::milliseconds>(batch_read_end - batch_read_start).count();

                std::cout << "[Batch] Processed " << batch_size << " frames in "
                          << batch_process_ms << "ms ("
                          << (batch_process_ms > 0 ? batch_size * 1000 / batch_process_ms : 0)
                          << " fps), total: " << frameId << std::endl;
            }
        } else {
            std::vector<std::string> image_files = listImageFiles(dataset_dir);
            for (const auto& filepath : image_files) {
                if (!processed_files.insert(filepath).second) continue;
                std::vector<unsigned char> buffer = readFileToBuffer(filepath);
                if (buffer.empty()) {
                    std::cout << "[Read][Fail] file=" << filepath << std::endl;
                    continue;
                }
                frameId++;
                new_frames++;
                total_read_bytes += buffer.size();
                CameraStreamData streams;
                streams[runtime_cfg.camera_id].push_back(buffer);
                TobaccoVisionSystem::generatePointCloud(streams, frameId);
                if (frameId % 20 == 0) std::cout << "." << std::flush;
            }
            if (new_frames > 0) {
                std::cout << "\n[Data] New frames: " << new_frames
                          << ", total: " << frameId << std::endl;
            }
        }

        if (frameId > 0 && frameId - last_publish_frame >= runtime_cfg.publish_every_frames) {
            int batch_frames = frameId - last_publish_frame;
            auto before_publish = std::chrono::steady_clock::now();
            auto ingest_ms = std::chrono::duration_cast<std::chrono::milliseconds>(before_publish - batch_start).count();
            std::cout << "================================================" << std::endl;
            publish_snapshot();
            auto after_publish = std::chrono::steady_clock::now();
            auto publish_ms = std::chrono::duration_cast<std::chrono::milliseconds>(after_publish - before_publish).count();
            auto total_batch_ms = std::chrono::duration_cast<std::chrono::milliseconds>(after_publish - batch_start).count();
            batch_seq++;
            std::cout << "[Batch] seq=" << batch_seq
                      << " frames=" << batch_frames
                      << " ingest_ms=" << ingest_ms
                      << " publish_ms=" << publish_ms
                      << " total_ms=" << total_batch_ms << std::endl;
            std::cout << "================================================" << std::endl;
            batch_start = after_publish;
        }

        if (!runtime_cfg.watch_mode && new_frames == 0) {
            if (runtime_cfg.use_batch_loader && loader && !loader->is_queue_empty()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                continue;
            }
            if (frameId > last_publish_frame) {
                std::cout << "================================================" << std::endl;
                publish_snapshot();
                std::cout << "================================================" << std::endl;
            }
            break;
        }

        if (runtime_cfg.watch_mode) {
            std::this_thread::sleep_for(std::chrono::milliseconds(runtime_cfg.scan_interval_ms));
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::seconds>(now - last_heartbeat).count() >= 5) {
            auto uptime_s = std::chrono::duration_cast<std::chrono::seconds>(now - service_start).count();
            std::cout << "[Heartbeat] uptime_s=" << uptime_s
                      << " total_frames=" << frameId
                      << " pending_publish_frames=" << (frameId - last_publish_frame) << std::endl;
            last_heartbeat = now;
        }
    }

    if (loader) {
        loader->stop();
        delete loader;
        std::cout << "[BatchLoader] Stopped" << std::endl;
    }

    ply_processor.stop();
    std::cout << "[PlyProcessor] Stopped" << std::endl;

    return 0;
}
