#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"

#include "dwa_critics/homing.hpp"

namespace dwa_critics
{
HomingCritic::HomingCritic()
: BaseCritic()
{
    name_ = "Homing";
}

void HomingCritic::on_initialize()
{
    auto node = parent_.lock();
    node->declare_parameter(name_ + ".weight", rclcpp::ParameterValue(1.0));
    weight_ = node->get_parameter(name_ + ".weight").as_double();
    node->declare_parameter(name_ + ".invert_score", rclcpp::ParameterValue(true));
    invert_score_ = node->get_parameter(name_ + ".invert_score").as_bool();
}

void HomingCritic::prepare(const nav_2d_msgs::msg::Path2D& /*global_path*/, const geometry_msgs::msg::Pose2D& goal_pose)
{
    goal_pose_ = goal_pose;
    unsigned int x, y;
    if (costmap_->worldToMap(goal_pose.x, goal_pose.y, x, y))
    {
        goal_inside_costmap_ = true;
    }
    else
    {
        goal_inside_costmap_ = false;
    }
}

double HomingCritic::scorePath(const nav_2d_msgs::msg::Path2D& local_path)
{
    if (goal_inside_costmap_)
    {
        return std::hypot(
            (goal_pose_.x - local_path.poses.back().x)*(goal_pose_.x - local_path.poses.back().x),
            (goal_pose_.y - local_path.poses.back().y)*(goal_pose_.y - local_path.poses.back().y));
    }
    else
    {
        return 0.0;
    }
}
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(dwa_critics::HomingCritic, dwa_critics::BaseCritic)