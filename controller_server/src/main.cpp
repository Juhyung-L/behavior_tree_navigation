#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "controller_server/controller_server.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<controller_server::ControllerServer>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}