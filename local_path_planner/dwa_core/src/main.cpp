#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "dwa_core/dwa_local_planner.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<dwa_core::DWALocalPlanner>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}