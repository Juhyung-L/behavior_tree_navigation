#include "bt_nodes/plugins/action/clear_costmap_service.hpp"

namespace bt_nodes
{
ClearEntireCostmapService::ClearEntireCostmapService(
    const std::string& service_name,
    const BT::NodeConfiguration& conf)
: BTServiceNode<nav2_msgs::srv::ClearEntireCostmap>(service_name, conf)
{}

void ClearEntireCostmapService::on_tick()
{
    increment_recovery_count();
}
}

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<bt_nodes::ClearEntireCostmapService>("ClearEntireCostmap");
}