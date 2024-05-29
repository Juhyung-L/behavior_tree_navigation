#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/node_utils.hpp"

#include "dwa_critics/obstacle_proximity.hpp"

namespace dwa_critics
{
ObstacleProximityCritic::ObstacleProximityCritic()
: BaseCritic()
{
    name_ = "ObstacleProximity";
}

void ObstacleProximityCritic::on_initialize()
{
    auto node = parent_.lock();

    nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + "." + name_ + ".weight", rclcpp::ParameterValue(1.0));
    nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + "." + name_ + ".invert_score", rclcpp::ParameterValue(true));
    
    weight_ = node->get_parameter(plugin_name_ + "." + name_ + ".weight").as_double();
    invert_score_ = node->get_parameter(plugin_name_ + "." + name_ + ".invert_score").as_bool();
}

void ObstacleProximityCritic::prepare(const nav_2d_msgs::msg::Path2D& /*globa_traj*/, const geometry_msgs::msg::Pose2D& /*goal_pose*/)
{}

double ObstacleProximityCritic::scorePath(const nav_2d_msgs::msg::Path2D& local_traj)
{
    double score = 0.0;
    for (size_t i=0; i<local_traj.poses.size(); ++i)
    {
        unsigned int x, y;
        if (costmap_->worldToMap(local_traj.poses[i].x, local_traj.poses[i].y, x, y))
        {
            if (costmap_->getCost(x, y) >= nav2_costmap_2d::LETHAL_OBSTACLE)
            {
                // if the current pose is inside an obstacle, treat the remaining pose as also being inside the obstacle
                // prevents trajectories that pass through obstacles from being favored
                score += static_cast<double>(local_traj.poses.size() - i) * static_cast<double>(nav2_costmap_2d::LETHAL_OBSTACLE);
                break;
            }
            else
            {
                score += static_cast<double>(costmap_->getCost(x, y));
            }
        }
    }
    return score;
}
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(dwa_critics::ObstacleProximityCritic, dwa_critics::BaseCritic)