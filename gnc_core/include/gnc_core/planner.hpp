#ifndef PLANNER_HPP_
#define PLANNER_HPP_

#include <memory>
#include <string>

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2_ros/buffer.h"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace gnc_core
{
class Planner
{
public:
    using Ptr = std::shared_ptr<Planner>;
    
    virtual ~Planner() {}
    virtual void configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent,
        std::shared_ptr<tf2_ros::Buffer> tf,
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) = 0;
    virtual void activate() = 0;
    virtual void deactivate() = 0;
    virtual void cleanup() = 0;

    // provide start and goal pose, return a path
    virtual nav_msgs::msg::Path computePath(
        const geometry_msgs::msg::PoseStamped& start,
        const geometry_msgs::msg::PoseStamped& goal) = 0;

protected:
    std::string plugin_name_;
};
}

#endif