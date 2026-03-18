#include <zmq.hpp>
#include <iostream>
#include <string>
#include "../include/json.hpp"

using json = nlohmann::json;

int main() {
    zmq::context_t context(1);
    zmq::socket_t subscriber(context, zmq::socket_type::sub);

    subscriber.connect("tcp://localhost:5555");
    subscriber.set(zmq::sockopt::subscribe, "DEFECTS");
    subscriber.set(zmq::sockopt::subscribe, "SUMMARY");
    subscriber.set(zmq::sockopt::subscribe, "DEPTH_ARRAY");

    std::cout << "ZMQ Subscriber started, listening on tcp://localhost:5555" << std::endl;
    std::cout << "Subscribed to: DEFECTS, SUMMARY, DEPTH_ARRAY" << std::endl;
    std::cout << "----------------------------------------" << std::endl;

    while (true) {
        zmq::message_t topic_msg;
        zmq::message_t data_msg;

        subscriber.recv(topic_msg, zmq::recv_flags::none);
        subscriber.recv(data_msg, zmq::recv_flags::none);

        std::string topic(static_cast<char*>(topic_msg.data()), topic_msg.size());
        std::string data(static_cast<char*>(data_msg.data()), data_msg.size());

        std::cout << "\n[" << topic << "]" << std::endl;

        try {
            if (topic == "DEPTH_ARRAY") {
            json msg = json::parse(data);
            std::cout << "  PLY: " << msg["ply_path"] << std::endl;
            std::cout << "  Grid: " << msg["grid_y"] << "x" << msg["grid_x"] << std::endl;
            std::cout << "  Array size: " << msg["depth_array"].size() << std::endl;

            auto arr = msg["depth_array"].get<std::vector<float>>();
            float sum = 0, min_val = 1e9, max_val = -1e9;
            int non_zero = 0;
            for (float v : arr) {
                sum += v;
                if (v > 0) {
                    min_val = std::min(min_val, v);
                    max_val = std::max(max_val, v);
                    non_zero++;
                }
            }
            std::cout << "  Depth range: [" << min_val << ", " << max_val << "]" << std::endl;
            std::cout << "  Average: " << sum / arr.size() << std::endl;
            std::cout << "  Non-zero points: " << non_zero << "/" << arr.size() << std::endl;

            // 显示前10个和后10个元素
            std::cout << "  First 10: ";
            for (int i = 0; i < 10 && i < arr.size(); ++i) {
                std::cout << arr[i] << " ";
            }
            std::cout << "\n  Last 10: ";
            for (int i = std::max(0, (int)arr.size() - 10); i < arr.size(); ++i) {
                std::cout << arr[i] << " ";
            }
            std::cout << std::endl;
        } else {
            json msg = json::parse(data);
            std::cout << msg.dump(2) << std::endl;
        }
        } catch (const std::exception& e) {
            std::cerr << "  Error: " << e.what() << std::endl;
        }

        std::cout << "----------------------------------------" << std::endl;
    }

    return 0;
}
