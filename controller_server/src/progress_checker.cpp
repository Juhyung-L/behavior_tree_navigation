#include "nav2_util/node_utils.hpp"

#include "controller_server/progress_checker.hpp"

namespace controller_server
{
void ProgressChecker::initialize(const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent)
{
    auto node = parent.lock();
    clock_ = node->get_clock();

    nav2_util::declare_parameter_if_not_declared(
        node, name_ + ".ProgressChecker.minimum_distance", rclcpp::ParameterValue(0.1));
    nav2_util::declare_parameter_if_not_declared(
        node, name_ + ".ProgressChecker.movement_time_allowance", rclcpp::ParameterValue(5.0));
    
    minimum_distance_ = node->get_parameter(name_ + ".ProgressChecker.minimum_distance").as_double();
    movement_time_allowance_ 
        = rclcpp::Duration::from_seconds(node->get_parameter(name_ + ".ProgressChecker.movement_time_allowance").as_double());
}

bool ProgressChecker::isProgressed(geometry_msgs::msg::Pose& cur_pose)
{
    if ((!baseline_set_) || movedEnough(cur_pose))
    {
        setBaseline(cur_pose);
        return true;
    }
    // if robot has not moved enough yet, check if time allowance has passed
    return (clock_->now() - baseline_time_) < movement_time_allowance_;
}

bool ProgressChecker::movedEnough(const geometry_msgs::msg::Pose& cur_pose)
{
    double dx = cur_pose.position.x - baseline_pose_.position.x;
    double dy = cur_pose.position.y - baseline_pose_.position.y;
    return std::hypot(dx, dy) > minimum_distance_;
}

void ProgressChecker::setBaseline(const geometry_msgs::msg::Pose& pose)
{
    baseline_pose_ = pose;
    baseline_set_ = true;
    baseline_time_ = clock_->now();
}

void ProgressChecker::reset()
{
    baseline_set_ = false;
}
}