#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "planner_server/planner_server.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<planner_server::PlannerServer>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}
