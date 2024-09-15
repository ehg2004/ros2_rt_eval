#include "rclcpp/rclcpp.hpp"
#include "ros2_rt_eval_dep/srv/vector.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>
#include <vector>
#include <string>
#include <fstream>

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    if (argc < 4) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Usage: vector_client <num_calls> <vector_size> <output_file.csv>");
        return 1;
    }

    int num_calls = std::atoi(argv[1]);
    int vector_size = std::atoi(argv[2]);
    std::string output_file = argv[3];

    auto node = rclcpp::Node::make_shared("vector_client");
    auto client = node->create_client<ros2_rt_eval_dep::srv::Vector>("vector_service");

    while (!client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return 0;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service not available, waiting again...");
    }

    std::vector<int64_t> latencies;
    auto request = std::make_shared<ros2_rt_eval_dep::srv::Vector::Request>();

    // Generate input vector based on the specified size
    request->input.resize(vector_size);
    for (int i = 0; i < vector_size; ++i) {
        request->input[i] = i + 1;  // Populate the vector with some values (1, 2, 3, ...)
    }
        rclcpp::sleep_for(std::chrono::milliseconds(300));

    for (int i = 0; i < num_calls; ++i) {
        
        auto start_time = std::chrono::steady_clock::now();

        auto result_future = client->async_send_request(request);

        if (rclcpp::spin_until_future_complete(node, result_future) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            auto end_time = std::chrono::steady_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();
            latencies.push_back(duration);

            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Iteration %d: Latency = %ld microseconds", i + 1, duration);
        }
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call vector_service");
            latencies.push_back(-1);  // Indicate failure with -1
        }
        rclcpp::sleep_for(std::chrono::microseconds(60));

    }

    // Write latencies to a CSV file
    std::ofstream output_csv(output_file);
    if (output_csv.is_open()) {
        output_csv << "Iteration,Latency (microseconds)\n";
        for (size_t i = 0; i < latencies.size(); ++i) {
            output_csv << i + 1 << "," << latencies[i] << "\n";
        }
        output_csv.close();
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Latencies recorded in %s", output_file.c_str());
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to open file: %s", output_file.c_str());
    }

    rclcpp::shutdown();
    return 0;
    //varias quantidade de dados 
    // teste real
    // procurar serviços teais
    //verde vermelho azul
    //100 mil amostras  
    //media maxima minima desvio padrão
    //over ethernet???
    //image sized vector???
    //image msg ros
    // aumentando linearmente???
    // tempo criação do nó ???
    // rt_clock sincronizado
}
