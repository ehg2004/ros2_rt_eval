#include "rclcpp/rclcpp.hpp"
#include "ros2_rt_eval_dep/srv/vector.hpp"  // Your custom service

using Vector = ros2_rt_eval_dep::srv::Vector;
using namespace std::placeholders;

class VectorService : public rclcpp::Node
{
public:
    VectorService() : Node("vector_service")
    {
        service_ = this->create_service<Vector>("vector_service", std::bind(&VectorService::handle_service, this, _1, _2));
    }

private:
    void handle_service(const std::shared_ptr<Vector::Request> request,
                        std::shared_ptr<Vector::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "Received request");
        response->output = request->input;  // Return the vector unaltered
    }

    rclcpp::Service<Vector>::SharedPtr service_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VectorService>());
    rclcpp::shutdown();
    return 0;
}
