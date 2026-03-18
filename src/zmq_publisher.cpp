#include "zmq_publisher.hpp"

ZmqPublisher::ZmqPublisher() : context(1), publisher(context, ZMQ_PUB) {}

ZmqPublisher::~ZmqPublisher() {
    publisher.close();
    context.close();
}

void ZmqPublisher::init(const std::string& address) {
    // 绑定到指定地址 (例如: tcp://*:5555)
    publisher.bind(address);
}

void ZmqPublisher::publish(const std::string& topic, const std::string& message) {
    // 发送主题 (带SNDMORE标志)
    publisher.send(zmq::const_buffer(topic.data(), topic.size()), zmq::send_flags::sndmore);
    // 发送消息内容
    publisher.send(zmq::const_buffer(message.data(), message.size()));
}
