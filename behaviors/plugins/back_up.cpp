#include <queue>
#include <mutex>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_costmap_2d/cost_values.hpp"

#include "behaviors/plugins/back_up.hpp"

namespace behaviors
{
BackUp::BackUp()
: TimedBehavior<Action>()
{
    behavior_name_ = "BackUp";
    action_name_ = "back_up";
}

void BackUp::onConfigure()
{
    auto node = parent_.lock();
    nav2_util::declare_parameter_if_not_declared(node, behavior_name_ + ".simulate_ahead_time", rclcpp::ParameterValue(0.5));
    simulate_ahead_time_ = node->get_parameter(behavior_name_ + ".simulate_ahead_time").as_double();
}

ResultStatus BackUp::onRun(const std::shared_ptr<const Action::Goal> command)
{
    if (!nav2_util::getCurrentPose(initial_pose_, *tf_, costmap_frame_, robot_frame_, transform_tolerance_))
    {
        RCLCPP_ERROR(logger_, "Unable to get robot pose.");
        return ResultStatus::FAILED;
    }

    back_up_direction_ = getDirectionToNearestObstacle(initial_pose_);
    // back up direction is the opposite of the direction to the closest obstacle
    back_up_direction_.x *= -1.0;
    back_up_direction_.y *= -1.0;

    end_time_ = clock_->now() + rclcpp::Duration(command->duration);
    return ResultStatus::SUCCEEDED;
}

ResultStatus BackUp::onCycleUpdate()
{
    auto time_left = end_time_ - clock_->now();
    if (time_left.seconds() < 0)
    {
        RCLCPP_ERROR(logger_, "Allowed time for %s passed. Aborting.", behavior_name_.c_str());
        return ResultStatus::FAILED;
    }

    auto goal = action_server_->get_current_goal();

    geometry_msgs::msg::PoseStamped current_pose;
    if (!nav2_util::getCurrentPose(current_pose, *tf_, costmap_frame_, robot_frame_, transform_tolerance_))
    {
        RCLCPP_ERROR(logger_, "Unable to get robot pose.");
        return ResultStatus::FAILED;
    }

    double dx = current_pose.pose.position.x - initial_pose_.pose.position.x;
    double dy = current_pose.pose.position.y - initial_pose_.pose.position.y;
    double distance = std::hypot(dx, dy);

    if (distance >= goal->back_up_distance)
    {
        RCLCPP_INFO(logger_, "%s complete.", behavior_name_.c_str());
        return ResultStatus::SUCCEEDED;
    }

    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = goal->back_up_speed * back_up_direction_.x;
    cmd_vel.linear.y = goal->back_up_speed * back_up_direction_.y;

    if (!isObstacleFree(cmd_vel, current_pose))
    {
        RCLCPP_ERROR(logger_, "Collision head. Canceling %s", behavior_name_.c_str());
        return ResultStatus::FAILED;
    }
    vel_pub_->publish(cmd_vel);
    return ResultStatus::RUNNING;
}

bool BackUp::isObstacleFree(const geometry_msgs::msg::Twist& cmd_vel, const geometry_msgs::msg::PoseStamped& current_pose)
{
    bool fetch_data = true;
    const int max_cycle_count = static_cast<int>(cycle_frequency_* simulate_ahead_time_);
    for (int i=0; i<max_cycle_count; ++i)
    {
        double elapsed_time = static_cast<double>(i) / cycle_frequency_;
        geometry_msgs::msg::Pose2D simulated_pose;
        simulated_pose.x = elapsed_time * cmd_vel.linear.x + current_pose.pose.position.x;
        simulated_pose.y = elapsed_time * cmd_vel.linear.y + current_pose.pose.position.y;

        if (!collision_checker_->isCollisionFree(simulated_pose, fetch_data))
        {
            return false;
        }
        fetch_data = false;
    }
    return true;
}

// bfs to find the closest obstacle cell
geometry_msgs::msg::Vector3 BackUp::getDirectionToNearestObstacle(const geometry_msgs::msg::PoseStamped& current_pose)
{
    costmap_ = costmap_sub_->getCostmap();
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap_->getMutex()));
    geometry_msgs::msg::Vector3 dir;

    unsigned int x_m, y_m;
    if (!costmap_->worldToMap(current_pose.pose.position.x, current_pose.pose.position.y, x_m, y_m))
    {
        RCLCPP_ERROR(logger_, "Robot is outside of local costmap.");
        return dir;
    }

    unsigned int obstacle_idx;
    const unsigned int size_x = costmap_->getSizeInCellsX();
    const unsigned int size_y = costmap_->getSizeInCellsY();
    std::vector<bool> visited(size_x * size_y, false);
    std::queue<unsigned int> q;
    q.emplace(costmap_->getIndex(x_m, y_m));
    while (!q.empty())
    {
        unsigned int idx = q.front();
        q.pop();
        costmap_->indexToCells(idx, x_m, y_m);
        
        // hit an obstacle
        if (costmap_->getCost(idx) > nav2_costmap_2d::MAX_NON_OBSTACLE)
        {
            obstacle_idx = idx;
            break;
        }

        unsigned int new_idx;
        if (x_m > 0) // left
        {
            new_idx = idx - 1;
            if (!visited[new_idx])
            {
                q.emplace(new_idx);
                visited[new_idx] = true;
            }
        }
        if (x_m < size_x - 1) // right
        {
            new_idx = idx + 1;
            if (!visited[new_idx])
            {
                q.emplace(new_idx);
                visited[new_idx] = true;
            }
        }
        if (y_m > 0) // down
        {
            new_idx = idx - size_x;
            if (!visited[new_idx])
            {
                q.emplace(new_idx);
                visited[new_idx] = true;
            }
        }
        if (y_m < size_y - 1) // up
        {
            new_idx = idx + size_x;
            if (!visited[new_idx])
            {
                q.emplace(new_idx);
                visited[new_idx] = true;
            }
        }
    }
    if (q.empty())
    {
        // didn't find any obstacle
        return dir;
    }

    costmap_->indexToCells(obstacle_idx, x_m, y_m);
    double x_w, y_w;
    costmap_->mapToWorld(x_m, y_m, x_w, y_w);

    dir.x = x_w - current_pose.pose.position.x;
    dir.y = y_w - current_pose.pose.position.y;
    // make into unit vecetor
    double mag = std::hypot(dir.x, dir.y);
    dir.x /= mag;
    dir.y /= mag;
    return dir;
}
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(behaviors::BackUp, gnc_core::Behavior)