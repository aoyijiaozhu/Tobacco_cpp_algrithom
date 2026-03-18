#include <zmq.hpp>
#include <iostream>
#include <fstream>
#include <string>
#include "../include/json.hpp"

using json = nlohmann::json;

int main() {
    zmq::context_t context(1);
    zmq::socket_t subscriber(context, zmq::socket_type::sub);

    subscriber.connect("tcp://localhost:5555");
    subscriber.set(zmq::sockopt::subscribe, "DEPTH_ARRAY");

    std::cout << "Listening for DEPTH_ARRAY messages..." << std::endl;

    while (true) {
        zmq::message_t topic_msg, data_msg;
        subscriber.recv(topic_msg, zmq::recv_flags::none);
        subscriber.recv(data_msg, zmq::recv_flags::none);

        std::string data(static_cast<char*>(data_msg.data()), data_msg.size());
        json msg = json::parse(data);

        auto arr = msg["depth_array"].get<std::vector<float>>();
        std::string ply_name = msg["ply_path"];

        std::string filename = "depth_array_" + std::to_string(arr.size()) + ".txt";
        std::ofstream out(filename);
        for (size_t i = 0; i < arr.size(); ++i) {
            out << arr[i];
            if ((i + 1) % 450 == 0) out << "\n";
            else out << " ";
        }
        out.close();

        std::cout << "Saved " << arr.size() << " values to " << filename << std::endl;
        std::cout << "  Range: [" << *std::min_element(arr.begin(), arr.end())
                  << ", " << *std::max_element(arr.begin(), arr.end()) << "]" << std::endl;
    }

    return 0;
}
