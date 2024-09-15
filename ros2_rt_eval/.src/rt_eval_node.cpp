#include "rclcpp/rclcpp.hpp"
#include "ros2_rt_eval_dep/srv/call_client_srv.hpp"
#include "ros2_rt_eval_dep/srv/vector.hpp"

#include <chrono>
#include <memory>
#include <vector>

using namespace std::chrono_literals;

void handle_service_request(
    const std::shared_ptr<ros2_rt_eval_dep::srv::CallClientSrv::Request> request,
    std::shared_ptr<ros2_rt_eval_dep::srv::CallClientSrv::Response> response,
    rclcpp::Client<ros2_rt_eval_dep::srv::Vector>::SharedPtr client)
{
    response->latencies.clear();
    response->latencies.reserve(request->num_calls);

    auto sample_request = std::make_shared<ros2_rt_eval_dep::srv::Vector::Request>();
    sample_request->input = {1, 2, 3, 4};  // Example input vector

    for (int i = 0; i < request->num_calls; ++i) {
        auto start = std::chrono::steady_clock::now();
        auto result_future = client->async_send_request(sample_request);

        if (rclcpp::spin_until_future_complete(client->get_node_base_interface(), result_future) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            auto end = std::chrono::steady_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
            response->latencies.push_back(duration);
        }
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call vector_service");
            response->latencies.push_back(-1);  // Indicate failure with -1
        }
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("call_client_service");

    auto client = node->create_client<ros2_rt_eval_dep::srv::Vector>("vector_service");
    while (!client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the vector service. Exiting.");
            return 0;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Vector service not available, waiting again...");
    }

    auto service = node->create_service<ros2_rt_eval_dep::srv::CallClientSrv>(
        "call_client_service", 
        std::bind(handle_service_request, std::placeholders::_1, std::placeholders::_2, client)
    );

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Call Client Service is ready.");

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
