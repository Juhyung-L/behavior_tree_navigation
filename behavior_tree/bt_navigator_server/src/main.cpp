#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "bt_navigator_server/bt_navigator_server.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<bt_navigator_server::BTNavigatorServer>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}