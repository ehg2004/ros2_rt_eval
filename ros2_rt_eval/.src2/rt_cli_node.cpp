#include "rclcpp/rclcpp.hpp"
#include "ros2_rt_eval_dep/srv/vector.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>
#include <vector>
#include <string>
#include <fstream>
#include <filesystem>

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    if (argc < 5) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Usage: vector_client <num_calls> <vector_size> <client_id> <dir>");
        return 1;
    }
    
    int num_calls = std::atoi(argv[1]);
    int vector_size = std::atoi(argv[2]);
    int client_id = std::atoi(argv[3]);
    std::string dir = argv[4];


    // auto node = rclcpp::Node::make_shared("vector_client"+std::to_string(client_id));
    auto node = rclcpp::Node::make_shared("vector_client");
    auto client = node->create_client<ros2_rt_eval_dep::srv::Vector>("vector_service");

    while (!client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return 0;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service not available, waiting again...");
    }

    // Create the directory if it doesn't exist
    std::filesystem::create_directories(dir);

    // CSV filename based on client ID and directory
    std::string filename = dir + "/" + std::to_string(client_id) + "client.csv";
    std::ofstream output_csv(filename);
    if (output_csv.is_open()) {
        output_csv << "Client ID,t1 (before service call),t2 (after service returns)\n";
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to open file: %s", filename.c_str());
        return 1;
    }

    // Prepare the input vector based on the specified size
    std::vector<int16_t> input_vector(vector_size);
    for (int i = 0; i < vector_size; ++i) {
        input_vector[i] = i + 1;  // Fill with numbers 1, 2, 3, ..., vector_size
    }
    std::vector<int16_t> id_vec(1);
    id_vec[0]=client_id;
    
    auto request = std::make_shared<ros2_rt_eval_dep::srv::Vector::Request>();
    request->input_vector = input_vector;
    request->client_id_vector = id_vec;  // Send the client ID as a vector

    for (int i = 0; i < num_calls; ++i) {
        // Record time t1
        auto t1 = std::chrono::steady_clock::now();

        auto result_future = client->async_send_request(request);

        if (rclcpp::spin_until_future_complete(node, result_future) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            // Record time t2
            auto t2 = std::chrono::steady_clock::now();

            auto t1_microseconds = std::chrono::duration_cast<std::chrono::microseconds>(t1.time_since_epoch()).count();
            auto t2_microseconds = std::chrono::duration_cast<std::chrono::microseconds>(t2.time_since_epoch()).count();

            // Log and record the result
            output_csv << client_id << "," << t1_microseconds << "," << t2_microseconds << "\n";
            //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "CLIENT Client ID: %d, t1: %ld, t2: %ld", client_id, t1_microseconds, t2_microseconds);
        } else {
            //it's just a fad.  And maybe those people use the scrollback code.CLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call vector_service");
        }

        rclcpp::sleep_for(std::chrono::microseconds(60));
    }

    output_csv.close();
    rclcpp::shutdown();
    return 0;
}
