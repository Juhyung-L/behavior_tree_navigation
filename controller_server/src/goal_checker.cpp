#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "nav2_util/node_utils.hpp"

#include "controller_server/goal_checker.hpp"

namespace controller_server
{
void GoalChecker::initialize(const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent)
{
    auto node = parent.lock();

    nav2_util::declare_parameter_if_not_declared(
        node, name_ + ".GoalChecker.xy_tolerance", rclcpp::ParameterValue(0.25));
    nav2_util::declare_parameter_if_not_declared(
        node, name_ + ".GoalChecker.yaw_tolerance", rclcpp::ParameterValue(0.25));
    
    xy_tolerance_ = node->get_parameter(name_ + ".GoalChecker.xy_tolerance").as_double();
    yaw_tolerance_ = node->get_parameter(name_ + ".GoalChecker.yaw_tolerance").as_double();
    xy_tolerance_sq_ = xy_tolerance_ * xy_tolerance_;
}

bool GoalChecker::isGoalReached(
    const geometry_msgs::msg::Pose& cur_pose,
    const geometry_msgs::msg::Pose& goal_pose)
{
    double dx = cur_pose.position.x - goal_pose.position.x;
    double dy = cur_pose.position.y - goal_pose.position.y;
    if ((dx*dx + dy*dy) > xy_tolerance_sq_)
    {
        return false;
    }
    double dyaw = angles::shortest_angular_distance(
        tf2::getYaw(cur_pose.orientation),
        tf2::getYaw(goal_pose.orientation));
    return fabs(dyaw) < yaw_tolerance_;
}
}