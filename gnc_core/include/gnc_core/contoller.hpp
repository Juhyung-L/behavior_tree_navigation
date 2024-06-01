#ifndef CONTROLLER_HPP_
#define CONTROLLER_HPP_

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/twist.hpp"
#include "pluginlib/class_loader.hpp"

namespace gnc_core
{
/**
 * @class Controller
 * @brief Base class for all controllers. All controllers must implement the computeVelocityCommand()
 * function, which computes the optimal velocity commands to follow the global path.
 * The ControllerServer class will have a gnc_core::Controller::Ptr member, which will point to the
 * loaded plugin, which is a derived class of gnc_core::Controller.
*/
class Controller
{
public:
    using Ptr = std::shared_ptr<gnc_core::Controller>;

    virtual ~Controller() {}
    virtual void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent,
        std::shared_ptr<tf2_ros::Buffer> tf,
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) = 0;
    virtual void activate() = 0;
    virtual void deactivate() = 0;
    virtual void cleanup() = 0;

    // provided robot's current pose and velocity, return a velocity command
    virtual geometry_msgs::msg::Twist computeVelocityCommand(
        const geometry_msgs::msg::Pose&  current_pose,
        const geometry_msgs::msg::Twist& current_velocity,
        const nav_msgs::msg::Path& global_path) = 0;

protected:
    std::string plugin_name_;
};
}

#endif