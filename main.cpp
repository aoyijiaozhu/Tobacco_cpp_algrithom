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

#include "zmq_publisher.hpp" 

namespace fs = std::filesystem;
using json = nlohmann::json;

bool natural_sort(const std::string& a, const std::string& b) {
    std::string fa = fs::path(a).filename().string();
    std::string fb = fs::path(b).filename().string();
    if (fa.length() != fb.length()) return fa.length() < fb.length();
    return fa < fb;
}

std::vector<unsigned char> readFileToBuffer(const std::string& path) {
    std::ifstream file(path, std::ios::binary);
    if (!file) return {};
    return std::vector<unsigned char>((std::istreambuf_iterator<char>(file)),
                                      std::istreambuf_iterator<char>());
}

struct RuntimeConfig {
    std::string dataset_dir = "";
    std::string zmq_bind = "tcp://*:5555";
    bool watch_mode = true;
    int scan_interval_ms = 500;
    int publish_every_frames = 20;
    int camera_id = 101;
    int batch_id_start = 20240101;
};

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
    }
    if (cfg.publish_every_frames <= 0) cfg.publish_every_frames = 1;
    if (cfg.scan_interval_ms < 50) cfg.scan_interval_ms = 50;
    return cfg;
}

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

int main(int argc, char** argv) {
    std::string config_path = "config.json";
    if (argc > 1) config_path = argv[1];

    std::cout << ">>> Start: Backend ZMQ Global Line-Scan System <<<" << std::endl;

    RuntimeConfig runtime_cfg = loadRuntimeConfig(config_path);
    TobaccoVisionSystem::init(config_path);

    ZmqPublisher zmq_pub;
    zmq_pub.init(runtime_cfg.zmq_bind);
    std::cout << "[ZMQ] Publisher bound to " << runtime_cfg.zmq_bind << std::endl;

    std::string dataset_dir = resolveDatasetDir(runtime_cfg.dataset_dir);
    if (dataset_dir.empty()) {
        std::cerr << "[Error] Dataset directory not found." << std::endl;
        return 1;
    }
    std::cout << "[Data] Using dataset dir: " << dataset_dir << std::endl;
    std::cout << "[Mode] " << (runtime_cfg.watch_mode ? "watch" : "oneshot") << std::endl;

    std::set<std::string> processed_files;
    int frameId = 0;
    int last_publish_frame = 0;
    int batch_id = runtime_cfg.batch_id_start;
    int batch_seq = 0;
    int read_fail_count = 0;
    std::uint64_t total_read_bytes = 0;
    auto service_start = std::chrono::steady_clock::now();
    auto last_heartbeat = service_start;
    auto batch_start = service_start;

    auto publish_snapshot = [&]() {
        // 先生成当前批次的全局 PLY 路径
        double length_m = TobaccoVisionSystem::getStitchedLength();
        double mass_kg = 0.0;
        double total_vol = TobaccoVisionSystem::calculateVolume("", mass_kg);
        std::string finalPlyPath = TobaccoVisionSystem::generateFullPly(batch_id++);
        std::string detectImagePath = "result_images/global_detect.jpg";

        // 全局缺陷检测（会写 global_detect.jpg）
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
        std::vector<std::string> image_files = listImageFiles(dataset_dir);
        int new_frames = 0;
        for (const auto& filepath : image_files) {
            if (!processed_files.insert(filepath).second) continue;
            std::vector<unsigned char> buffer = readFileToBuffer(filepath);
            if (buffer.empty()) {
                read_fail_count++;
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
            auto read_mb = static_cast<double>(total_read_bytes) / (1024.0 * 1024.0);
            std::cout << "[Heartbeat] uptime_s=" << uptime_s
                      << " total_frames=" << frameId
                      << " processed_files=" << processed_files.size()
                      << " pending_publish_frames=" << (frameId - last_publish_frame)
                      << " read_mb=" << read_mb
                      << " read_fail=" << read_fail_count << std::endl;
            last_heartbeat = now;
        }
    }
    return 0;
}
