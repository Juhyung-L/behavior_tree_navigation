#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "global_path_planner/a_star_node.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<a_star::AStar>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}