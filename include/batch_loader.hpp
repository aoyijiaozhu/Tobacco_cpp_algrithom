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

/**
 * @brief 批次数据结构
 * @details 存储一批已解码的图像及其元数据
 */
struct BatchData {
    std::vector<cv::Mat> images;          // 已解码的图像列表
    std::vector<std::string> filenames;   // 对应的文件名列表
    std::vector<int> frame_ids;           // 对应的帧编号列表
};

/**
 * @brief 批量加载器 - 多线程图像加载
 * @details 使用生产者-消费者模式，多线程扫描目录并解码图像
 */
class BatchLoader {
public:
    /**
     * @brief 构造函数
     * @param folder_path 图像目录路径
     * @param batch_size 每批图像数量
     */
    BatchLoader(std::string folder_path, int batch_size);

    /**
     * @brief 析构函数
     * @details 自动停止所有线程
     */
    ~BatchLoader();

    /**
     * @brief 启动加载器
     * @details 启动目录扫描线程和4个解码工作线程
     */
    void start();

    /**
     * @brief 停止加载器
     * @details 停止所有线程并清理资源
     */
    void stop();

    /**
     * @brief 获取一批数据
     * @param out_batch 输出批次数据
     * @param timeout_ms 超时时间 (毫秒)，-1表示无限等待
     * @return bool 是否成功获取 (false表示超时或已停止)
     */
    bool get_batch(BatchData& out_batch, int timeout_ms = 100);

    /**
     * @brief 检查队列是否为空
     * @return bool 队列是否为空
     */
    bool is_queue_empty();

private:
    /**
     * @brief 目录扫描主循环
     * @details 扫描新文件并加入待处理队列
     */
    void worker_loop();

    /**
     * @brief 图像加载工作线程
     * @details 从待处理队列取文件，解码后组装批次
     */
    void loader_worker();

    std::string folder_path;              // 监控的目录路径
    int batch_size;                       // 批次大小

    std::atomic<bool> running;            // 运行状态标志
    std::thread worker_thread;            // 目录扫描线程
    std::vector<std::thread> loader_threads;  // 解码工作线程池 (4个)

    std::mutex queue_mutex;               // 批次队列互斥锁
    std::condition_variable queue_cv;     // 批次队列条件变量
    std::queue<BatchData> batch_queue;    // 已完成的批次队列

    std::mutex files_mutex;               // 文件队列互斥锁
    std::queue<std::string> pending_files;    // 待处理文件队列
    std::set<std::string> processed_files;    // 已处理文件集合

    std::mutex buffer_mutex;              // 组装缓冲区互斥锁
    BatchData assembly_buffer;            // 批次组装缓冲区
};
