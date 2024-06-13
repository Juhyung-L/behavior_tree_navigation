#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "behaviors/behavior_server.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<behavior_server::BehaviorServer>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}