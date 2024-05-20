#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "bt_navigator/bt_navigator_node.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<bt_navigator::BTNavigatorNode>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}