#ifndef GOAL_CHECKER_HPP_
#define GOAL_CHECKER_HPP_

#include <string>

#include "geometry_msgs/msg/pose.hpp"
#include "angles/angles.h"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2/utils.h"

namespace controller_server
{
class GoalChecker
{
public:
    void initialize(const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent);
    bool isGoalReached(
        const geometry_msgs::msg::Pose& cur_pose,
        const geometry_msgs::msg::Pose& goal_pose);

private:
    double xy_tolerance_;
    double xy_tolerance_sq_;
    double yaw_tolerance_;
    std::string name_;
};
}

#endif