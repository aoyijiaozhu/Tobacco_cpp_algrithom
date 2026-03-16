#pragma once
#include <string>
#include "zmq.hpp"

class ZmqPublisher {
public:
    ZmqPublisher();
    ~ZmqPublisher();

    void init(const std::string& address);
    void publish(const std::string& topic, const std::string& message);

private:
    zmq::context_t context;
    zmq::socket_t publisher;
};
