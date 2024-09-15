#include "rclcpp/rclcpp.hpp"
#include "ros2_rt_eval_dep/srv/vector.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>
#include <vector>
#include <string>
#include <sstream>

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    if (argc < 2) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Usage: vector_client <int16...>");
        return 1;
    }

    auto node = rclcpp::Node::make_shared("vector_client");
    auto client = node->create_client<ros2_rt_eval_dep::srv::Vector>("vector_service");

    auto request = std::make_shared<ros2_rt_eval_dep::srv::Vector::Request>();
    request->input.reserve(argc - 1);  // Reserve space for arguments

    for (int i = 1; i < argc; ++i) {
        try {
            int16_t value = std::stoi(argv[i]);
            request->input.push_back(value);
        } catch (const std::exception &e) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Invalid input: %s", e.what());
            return 1;
        }
    }

    while (!client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return 0;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service not available, waiting again...");
    }

    auto start = std::chrono::steady_clock::now();

    auto result_future = client->async_send_request(request);

    if (rclcpp::spin_until_future_complete(node, result_future) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        auto end = std::chrono::steady_clock::now();
        auto response = result_future.get();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received response with output size: %zu", response->output.size());
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Time taken for service call: %ld microseconds", duration);
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service vector_service");
    }

    rclcpp::shutdown();
    return 0;
}
