#include "zmq_publisher.hpp"

ZmqPublisher::ZmqPublisher() : context(1), publisher(context, ZMQ_PUB) {}

ZmqPublisher::~ZmqPublisher() {
    publisher.close();
    context.close();
}

void ZmqPublisher::init(const std::string& address) {
    publisher.bind(address);
}

void ZmqPublisher::publish(const std::string& topic, const std::string& message) {
    publisher.send(zmq::const_buffer(topic.data(), topic.size()), zmq::send_flags::sndmore);
    publisher.send(zmq::const_buffer(message.data(), message.size()));
}
