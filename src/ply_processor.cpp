#include "ply_processor.hpp"
#include "zmq_publisher.hpp"
#include "json.hpp"
#include <fstream>
#include <sstream>
#include <algorithm>
#include <cmath>
#include <iostream>

using json = nlohmann::json;

PlyProcessor::PlyProcessor(ZmqPublisher* zmq_pub, float y_min, float y_max,
                           int grid_y, int grid_x)
    : zmq_pub_(zmq_pub), y_min_(y_min), y_max_(y_max),
      grid_y_(grid_y), grid_x_(grid_x), running_(false) {}

PlyProcessor::~PlyProcessor() {
    stop();
}

void PlyProcessor::start() {
    if (running_) return;
    running_ = true;
    worker_thread_ = std::thread(&PlyProcessor::worker_loop, this);
}

void PlyProcessor::stop() {
    running_ = false;
    queue_cv_.notify_all();
    if (worker_thread_.joinable()) {
        worker_thread_.join();
    }
}

void PlyProcessor::submit(const std::string& ply_path) {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    ply_queue_.push(ply_path);
    queue_cv_.notify_one();
}

void PlyProcessor::worker_loop() {
    while (running_) {
        std::unique_lock<std::mutex> lock(queue_mutex_);
        queue_cv_.wait(lock, [this] { return !ply_queue_.empty() || !running_; });

        if (!running_) break;

        if (!ply_queue_.empty()) {
            std::string ply_path = ply_queue_.front();
            ply_queue_.pop();
            lock.unlock();

            try {
                // 处理PLY文件
                auto depth_array = process_ply(ply_path);

                // 构造JSON消息
                json msg;
                msg["ply_path"] = ply_path;
                msg["grid_y"] = grid_y_;
                msg["grid_x"] = grid_x_;
                msg["depth_array"] = depth_array;

                // 发布到ZMQ
                zmq_pub_->publish("DEPTH_ARRAY", msg.dump());
                std::cout << "[PlyProcessor] Published depth array for " << ply_path << std::endl;
            } catch (const std::exception& e) {
                std::cerr << "[PlyProcessor] Error: " << e.what() << std::endl;
            }
        }
    }
}

std::vector<float> PlyProcessor::process_ply(const std::string& ply_path) {
    std::ifstream file(ply_path);
    if (!file.is_open()) {
        throw std::runtime_error("Cannot open PLY file: " + ply_path);
    }

    // 读取PLY头部，获取顶点数量
    std::string line;
    int vertex_count = 0;
    bool header_end = false;
    while (std::getline(file, line)) {
        if (line.find("element vertex") != std::string::npos) {
            std::istringstream iss(line);
            std::string tmp1, tmp2;
            iss >> tmp1 >> tmp2 >> vertex_count;
        }
        if (line.find("end_header") != std::string::npos) {
            header_end = true;
            break;
        }
    }

    if (!header_end || vertex_count == 0) {
        throw std::runtime_error("Invalid PLY format");
    }

    // 读取点云数据
    std::vector<float> x_coords, y_coords, z_coords;
    x_coords.reserve(vertex_count);
    y_coords.reserve(vertex_count);
    z_coords.reserve(vertex_count);

    for (int i = 0; i < vertex_count; ++i) {
        float x, y, z;
        if (!(file >> x >> y >> z)) break;
        x_coords.push_back(x);
        y_coords.push_back(y);
        z_coords.push_back(z);
    }
    file.close();

    if (x_coords.empty()) {
        throw std::runtime_error("No points in PLY file");
    }

    // 第一次原点偏移
    float x_min = *std::min_element(x_coords.begin(), x_coords.end());
    float y_min = *std::min_element(y_coords.begin(), y_coords.end());
    for (size_t i = 0; i < x_coords.size(); ++i) {
        x_coords[i] -= x_min;
        y_coords[i] -= y_min;
    }

    // Y轴裁剪
    std::vector<float> x_filtered, y_filtered, z_filtered;
    for (size_t i = 0; i < y_coords.size(); ++i) {
        if (y_coords[i] >= y_min_ && y_coords[i] <= y_max_) {
            x_filtered.push_back(x_coords[i]);
            y_filtered.push_back(y_coords[i]);
            z_filtered.push_back(z_coords[i]);
        }
    }

    if (x_filtered.empty()) {
        throw std::runtime_error("No points after Y-axis cropping");
    }

    // 第二次原点偏移
    x_min = *std::min_element(x_filtered.begin(), x_filtered.end());
    y_min = *std::min_element(y_filtered.begin(), y_filtered.end());
    for (size_t i = 0; i < x_filtered.size(); ++i) {
        x_filtered[i] -= x_min;
        y_filtered[i] -= y_min;
    }

    // 网格映射
    float y_max_val = *std::max_element(y_filtered.begin(), y_filtered.end());
    float res = (y_max_val + 1e-8f) / grid_y_;

    std::vector<float> depth(grid_y_ * grid_x_, 0.0f);
    std::vector<int> counts(grid_y_ * grid_x_, 0);

    for (size_t i = 0; i < x_filtered.size(); ++i) {
        int y_idx = std::min(static_cast<int>(y_filtered[i] / res), grid_y_ - 1);
        int x_idx = static_cast<int>(x_filtered[i] / res);
        if (x_idx < grid_x_) {
            int idx = y_idx * grid_x_ + x_idx;
            depth[idx] += z_filtered[i];
            counts[idx]++;
        }
    }

    // 计算平均深度
    for (size_t i = 0; i < depth.size(); ++i) {
        if (counts[i] > 0) {
            depth[i] /= counts[i];
        }
    }

    return depth;
}
