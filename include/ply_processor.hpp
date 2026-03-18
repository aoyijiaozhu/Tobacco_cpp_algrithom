#pragma once
#include <string>
#include <vector>
#include <mutex>
#include <thread>
#include <queue>
#include <atomic>
#include <condition_variable>

class ZmqPublisher;

/**
 * @brief PLY点云处理器
 * @details 异步处理PLY文件，转换为深度数组并通过ZMQ发布
 */
class PlyProcessor {
public:
    /**
     * @brief 构造函数
     * @param zmq_pub ZMQ发布器指针
     * @param y_min Y轴裁剪最小值 (米)
     * @param y_max Y轴裁剪最大值 (米)
     * @param grid_y 网格行数
     * @param grid_x 网格列数
     */
    PlyProcessor(ZmqPublisher* zmq_pub, float y_min = 0.3f, float y_max = 1.65f,
                 int grid_y = 20, int grid_x = 450);

    ~PlyProcessor();

    /**
     * @brief 启动处理线程
     */
    void start();

    /**
     * @brief 停止处理线程
     */
    void stop();

    /**
     * @brief 提交PLY文件进行处理
     * @param ply_path PLY文件路径
     */
    void submit(const std::string& ply_path);

private:
    /**
     * @brief 工作线程主循环
     */
    void worker_loop();

    /**
     * @brief 处理单个PLY文件
     * @param ply_path PLY文件路径
     * @return std::vector<float> 深度数组
     */
    std::vector<float> process_ply(const std::string& ply_path);

    ZmqPublisher* zmq_pub_;
    float y_min_, y_max_;
    int grid_y_, grid_x_;

    std::atomic<bool> running_;
    std::thread worker_thread_;

    std::mutex queue_mutex_;
    std::condition_variable queue_cv_;
    std::queue<std::string> ply_queue_;
};
