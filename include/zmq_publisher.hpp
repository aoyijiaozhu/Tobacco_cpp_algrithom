#pragma once
#include <string>
#include "zmq.hpp"

/**
 * @brief ZeroMQ消息发布器
 * @details 用于向后端发布检测结果和统计信息
 */
class ZmqPublisher {
public:
    /**
     * @brief 构造函数
     */
    ZmqPublisher();

    /**
     * @brief 析构函数
     */
    ~ZmqPublisher();

    /**
     * @brief 初始化发布器
     * @param address 绑定地址 (例如: "tcp://0.0.0.0:5555")
     */
    void init(const std::string& address);

    /**
     * @brief 发布消息
     * @param topic 主题名称 (例如: "DEFECTS", "SUMMARY")
     * @param message 消息内容 (通常为JSON字符串)
     */
    void publish(const std::string& topic, const std::string& message);

private:
    zmq::context_t context;   // ZMQ上下文
    zmq::socket_t publisher;  // 发布套接字
};
