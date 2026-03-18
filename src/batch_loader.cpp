#include "batch_loader.hpp"
#include <filesystem>
#include <algorithm>
#include <iostream>
#include <chrono>

namespace fs = std::filesystem;

// 自然排序：按文件名长度和字典序排序
static bool natural_sort(const std::string& a, const std::string& b) {
    std::string fa = fs::path(a).filename().string();
    std::string fb = fs::path(b).filename().string();
    if (fa.length() != fb.length()) return fa.length() < fb.length();
    return fa < fb;
}

BatchLoader::BatchLoader(std::string folder_path, int batch_size)
    : folder_path(folder_path), batch_size(batch_size), running(false) {}

BatchLoader::~BatchLoader() {
    stop();
}

void BatchLoader::start() {
    if (running) return;
    running = true;

    // 启动目录扫描线程
    worker_thread = std::thread(&BatchLoader::worker_loop, this);

    // 启动4个解码工作线程
    for (int i = 0; i < 4; ++i) {
        loader_threads.emplace_back(&BatchLoader::loader_worker, this);
    }
}

void BatchLoader::stop() {
    running = false;
    queue_cv.notify_all();

    if (worker_thread.joinable()) {
        worker_thread.join();
    }

    for (auto& t : loader_threads) {
        if (t.joinable()) t.join();
    }
    loader_threads.clear();
}

bool BatchLoader::get_batch(BatchData& out_batch, int timeout_ms) {
    std::unique_lock<std::mutex> lock(queue_mutex);

    // 无限等待模式
    if (timeout_ms < 0) {
        queue_cv.wait(lock, [this] { return !batch_queue.empty() || !running; });
        if (!batch_queue.empty()) {
            out_batch = batch_queue.front();
            batch_queue.pop();
            return true;
        }
        return false;
    }

    // 超时等待模式
    if (queue_cv.wait_for(lock, std::chrono::milliseconds(timeout_ms),
        [this] { return !batch_queue.empty(); })) {
        out_batch = batch_queue.front();
        batch_queue.pop();
        return true;
    }
    return false;
}

bool BatchLoader::is_queue_empty() {
    std::unique_lock<std::mutex> lock(queue_mutex);
    return batch_queue.empty();
}

void BatchLoader::worker_loop() {
    while (running) {
        std::vector<std::string> new_files;
        // 扫描目录查找新文件
        if (fs::exists(folder_path)) {
            for (const auto& entry : fs::directory_iterator(folder_path)) {
                if (!entry.is_regular_file()) continue;
                std::string path = entry.path().string();
                std::string ext = entry.path().extension().string();
                std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);

                // 只处理PNG和JPG文件
                if (ext == ".png" || ext == ".jpg") {
                    std::lock_guard<std::mutex> lock(files_mutex);
                    if (processed_files.find(path) == processed_files.end()) {
                        new_files.push_back(path);
                        processed_files.insert(path);
                    }
                }
            }
        }

        // 将新文件排序后加入待处理队列
        if (!new_files.empty()) {
            std::sort(new_files.begin(), new_files.end(), natural_sort);
            std::lock_guard<std::mutex> lock(files_mutex);
            for (const auto& f : new_files) {
                pending_files.push(f);
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
}

void BatchLoader::loader_worker() {
    while (running) {
        std::string file_to_load;
        {
            std::lock_guard<std::mutex> lock(files_mutex);
            if (!pending_files.empty()) {
                file_to_load = pending_files.front();
                pending_files.pop();
            }
        }

        if (file_to_load.empty()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            continue;
        }

        cv::Mat img = cv::imread(file_to_load, cv::IMREAD_UNCHANGED);
        if (img.empty()) continue;

        int frame_id = 0;
        try {
            std::string stem = fs::path(file_to_load).stem().string();
            std::string num;
            for(char c : stem) if(isdigit(c)) num += c;
            if(!num.empty()) frame_id = std::stoi(num);
        } catch(...) {}

        {
            std::lock_guard<std::mutex> lock(buffer_mutex);
            assembly_buffer.images.push_back(img);
            assembly_buffer.filenames.push_back(file_to_load);
            assembly_buffer.frame_ids.push_back(frame_id);

            if (assembly_buffer.images.size() >= (size_t)batch_size) {
                std::lock_guard<std::mutex> q_lock(queue_mutex);
                batch_queue.push(assembly_buffer);
                queue_cv.notify_one();

                std::cout << "[BatchLoader] Pushed Batch (Size " << batch_size << ")" << std::endl;

                assembly_buffer.images.clear();
                assembly_buffer.filenames.clear();
                assembly_buffer.frame_ids.clear();
            }
        }
    }
}
