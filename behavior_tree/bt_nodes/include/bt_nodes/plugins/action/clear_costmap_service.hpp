#ifndef CLEAR_COSTMAP_SERVICE_HPP_
#define CLEAR_COSTMAP_SERVICE_HPP_

#include <string>

#include "nav2_msgs/srv/clear_entire_costmap.hpp"

#include "bt_nodes/bt_service_node.hpp"

namespace bt_nodes
{
class ClearEntireCostmapService : public BTServiceNode<nav2_msgs::srv::ClearEntireCostmap>
{
public:
    ClearEntireCostmapService(
        const std::string& service_name,
        const BT::NodeConfiguration& conf);
    
    void on_tick() override;
};
}

#endif