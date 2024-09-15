#include "rclcpp/rclcpp.hpp"
#include "ros2_rt_eval_dep/srv/vector.hpp"

#include <chrono>
#include <fstream>
#include <string>
#include <filesystem>

void handle_request(
    const std::shared_ptr<ros2_rt_eval_dep::srv::Vector::Request> request,
    std::shared_ptr<ros2_rt_eval_dep::srv::Vector::Response> response,
    const std::string& dir)
{
    // Record time after request is received
    auto t1 = std::chrono::steady_clock::now();

    // Process the request (unmodified vector is sent back)
    response->output_vector = request->input_vector;
    //response->output_vector.operator=
    // Record time before response is sent
    auto t2 = std::chrono::steady_clock::now();
    //retornar de volta o t2
    //cenarios


    // Get the client ID from the request
    int client_id = request->client_id_vector[0];

    // CSV filename based on client ID and directory
    std::string filename = dir + "/" + std::to_string(client_id) + "server.csv";
    std::ofstream output_csv(filename, std::ios_base::app);  // Append to the file
    if (output_csv.is_open()) {
        auto t1_microseconds = std::chrono::duration_cast<std::chrono::microseconds>(t1.time_since_epoch()).count();
        auto t2_microseconds = std::chrono::duration_cast<std::chrono::microseconds>(t2.time_since_epoch()).count();

        output_csv << client_id << "," << t1_microseconds << "," << t2_microseconds << "\n";
        //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "SERVER Client ID: %d, t1: %ld, t2: %ld", client_id, t1_microseconds, t2_microseconds);
    } else {
        //RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to open file: %s", filename.c_str());
        //ROSINFO??
    }

}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    if (argc != 2) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Usage: vector_server <dir>");
        return 1;
    }

    std::string dir = argv[1];

    // Create the directory if it doesn't exist
    std::filesystem::create_directories(dir);

    auto node = rclcpp::Node::make_shared("vector_server");

    auto service = node->create_service<ros2_rt_eval_dep::srv::Vector>(
        "vector_service", 
        [dir](const std::shared_ptr<ros2_rt_eval_dep::srv::Vector::Request> request,
              std::shared_ptr<ros2_rt_eval_dep::srv::Vector::Response> response) 
        {
            handle_request(request, response, dir);
        }
    );

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Server Test is ready to process requests.");
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
