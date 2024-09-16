#include "rclcpp/rclcpp.hpp"
#include "ros2_rt_eval_dep/srv/vector.hpp"

#include <chrono>
#include <fstream>
#include <string>

void handle_request(
    const std::shared_ptr<ros2_rt_eval_dep::srv::Vector::Request> request,
    std::shared_ptr<ros2_rt_eval_dep::srv::Vector::Response> response)
{
    // Record time after request is received
    auto t2 = std::chrono::steady_clock::now();
    response->t2 = std::chrono::duration_cast<std::chrono::nanoseconds>(t2.time_since_epoch()).count();

    // Process the request (unmodified vector is sent back)
    response->output_vector = request->input_vector;
    
    // Record time before response is sent
    auto t3 = std::chrono::steady_clock::now();
    response->t3 = std::chrono::duration_cast<std::chrono::nanoseconds>(t3.time_since_epoch()).count();
    //retornar de volta o t2
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    if (argc < 1) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Usage: vector_server ");
        return 1;
    }

    // std::string num = std::string(argv[1]);

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("vector_server");

    rclcpp::Service<ros2_rt_eval_dep::srv::Vector>::SharedPtr service = 
    node->create_service<ros2_rt_eval_dep::srv::Vector>(
        "vector_service",&handle_request
    );

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Server Test is ready to process requests.");
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
