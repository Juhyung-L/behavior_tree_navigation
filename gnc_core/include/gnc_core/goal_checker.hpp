#ifndef GOAL_CHECKER_HPP_
#define GOAL_CHECKER_HPP_

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

namespace gnc_core
{
/**
 * @class GoalChecker
 * @brief Base class for goal checkers. All goal checkers must implement the isGoalReached() function,
 * which determines if the robot is at the goal position
*/
class GoalChecker
{
public:
    typedef std::shared_ptr<gnc_core::GoalChecker> Ptr;

    virtual ~GoalChecker() {}

    virtual void initialize(const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent,
        const std::string& plugin_name,
        const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) = 0;

    virtual void reset() = 0;

    virtual bool isGoalReached(const geometry_msgs::msg::Pose& query_pose, 
        const geometry_msgs::msg::Pose& goal_pose,
        const geometry_msgs::msg::Twist& velocity) = 0;

    virtual bool getTolerances(geometry_msgs::msg::Pose& pose_tolerance,
        geometry_msgs::msg::Twist& vel_tolerance) = 0;
};
}

#endif