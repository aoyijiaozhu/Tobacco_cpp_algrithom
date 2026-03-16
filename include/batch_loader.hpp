#pragma once
#include <string>
#include <vector>
#include <set>
#include <queue>
#include <mutex>
#include <thread>
#include <condition_variable>
#include <atomic>
#include <opencv2/opencv.hpp>

struct BatchData {
    std::vector<cv::Mat> images;
    std::vector<std::string> filenames;
    std::vector<int> frame_ids;
};

class BatchLoader {
public:
    BatchLoader(std::string folder_path, int batch_size);
    ~BatchLoader();

    void start();
    void stop();
    
    // Returns true if a batch was retrieved, false if timeout
    bool get_batch(BatchData& out_batch, int timeout_ms = 100);
    
    bool is_queue_empty();

private:
    void worker_loop();
    void loader_worker(); // 多线程加载函数

    std::string folder_path;
    int batch_size;
    
    std::atomic<bool> running;
    std::thread worker_thread;
    std::vector<std::thread> loader_threads; // 加载线程池
    
    std::mutex queue_mutex;
    std::condition_variable queue_cv;
    std::queue<BatchData> batch_queue;
    
    // 多线程读取相关
    std::mutex files_mutex;
    std::queue<std::string> pending_files;
    std::set<std::string> processed_files;
    
    // 批次组装缓冲区
    std::mutex buffer_mutex;
    BatchData assembly_buffer;
};
