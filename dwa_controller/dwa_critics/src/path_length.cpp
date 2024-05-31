#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_util/node_utils.hpp"

#include "dwa_critics/path_length.hpp"

namespace dwa_critics
{
PathLengthCritic::PathLengthCritic()
: BaseCritic()
{
    name_ = "PathLength";
}

void PathLengthCritic::on_initialize()
{
    auto node = parent_.lock();
    nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + "." + name_ + ".weight", rclcpp::ParameterValue(1.0));
    nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + "." + name_ + ".invert_score", rclcpp::ParameterValue(false));
    
    weight_ = node->get_parameter(plugin_name_ + "." + name_ + ".weight").as_double();
    invert_score_ = node->get_parameter(plugin_name_ + "." + name_ + ".invert_score").as_bool();
}

void PathLengthCritic::prepare(const nav_2d_msgs::msg::Path2D& /*global_path*/)
{}

double PathLengthCritic::scorePath(const nav_2d_msgs::msg::Path2D& local_path)
{
    double path_length = 0.0;
    for (size_t i=1; i<local_path.poses.size(); ++i)
    {
        double dx = local_path.poses[i].x - local_path.poses[i-1].x;
        double dy = local_path.poses[i].y - local_path.poses[i-1].y;
        path_length += (dx*dx) + (dy*dy);
    }
    return path_length;
}
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(dwa_critics::PathLengthCritic, dwa_critics::BaseCritic)