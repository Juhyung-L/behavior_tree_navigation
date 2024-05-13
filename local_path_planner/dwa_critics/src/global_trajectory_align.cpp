#include <limits>

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"

#include "dwa_critics/global_trajectory_align.hpp"

namespace dwa_critics
{ 
GlobalTrajectoryAlignCritic::GlobalTrajectoryAlignCritic()
: BaseCritic()
{
    name_ = "GlobalTrajectoryAlign";
}

void GlobalTrajectoryAlignCritic::on_initialize()
{
    nh_->declare_parameter(name_ + ".weight", rclcpp::ParameterValue(1.0));
    weight_ = nh_->get_parameter(name_ + ".weight").as_double();
    nh_->declare_parameter(name_ + ".invert_score", rclcpp::ParameterValue(true));
    invert_score_ = nh_->get_parameter(name_ + ".invert_score").as_bool();

    size_x_ = costmap_->getSizeInCellsX();
    size_y_ = costmap_->getSizeInCellsY();
}

void GlobalTrajectoryAlignCritic::prepare(const nav_2d_msgs::msg::Path2D& global_traj, const geometry_msgs::msg::Pose2D& /*goal_pose*/)
{
    makeAlignmentMap(global_traj);
}

double GlobalTrajectoryAlignCritic::scoreTrajectory(const nav_2d_msgs::msg::Path2D& local_traj)
{
    double score = 0.0;
    for (auto& pose : local_traj.poses)
    {
        unsigned int x, y;
        if (costmap_->worldToMap(pose.x, pose.y, x, y))
        {
            unsigned int idx = costmap_->getIndex(x, y);
            score += static_cast<double>(alignment_map_[idx]);
        }
    }
    return score;
}

void GlobalTrajectoryAlignCritic::makeAlignmentMap(const nav_2d_msgs::msg::Path2D& traj)
{
    alignment_map_.resize(size_x_ * size_y_);
    std::vector<bool>visited(size_x_*size_y_, false);
    std::queue<Node> q;

    // first push all trajectory cells into queue
    unsigned char val = std::numeric_limits<unsigned char>::min();
    for (auto rit=traj.poses.rbegin(); rit!=traj.poses.rend(); ++rit)
    {
        unsigned int x, y, idx;
        if (costmap_->worldToMap(rit->x, rit->y, x, y))
        {
            idx = costmap_->getIndex(x, y);
            q.emplace(idx, val);
            visited[idx] = true;
        }
        
        if (val < std::numeric_limits<unsigned char>::max())
        {
            ++val;
        }
    }

    // level-order traversal
    while (!q.empty())
    {
        int qsize = q.size();
        for (int i=0; i<qsize; ++i)
        {
            Node node = q.front();
            q.pop();
            
            unsigned int idx = node.idx;
            unsigned char val = node.parent_val;
            if (val < std::numeric_limits<unsigned char>::max())
            {
                ++val;
            }
            alignment_map_[idx] = val;
            unsigned int x, y;
            costmap_->indexToCells(idx, x, y);

            // 4-way neighbor search
            unsigned int new_idx;
            if (x > 0) // left
            {
                new_idx = idx - 1;
                if (!visited[new_idx])
                {
                    q.emplace(new_idx, val);
                    visited[new_idx] = true;
                }
            }
            if (x < size_x_ - 1) // right
            {
                new_idx = idx + 1;
                if (!visited[new_idx])
                {
                    q.emplace(new_idx, val);
                    visited[new_idx] = true;
                }
            }
            if (y > 0) // down
            {
                new_idx = idx - size_x_;
                if (!visited[new_idx])
                {
                    q.emplace(new_idx, val);
                    visited[new_idx] = true;
                }
            }
            if (y < size_y_ - 1) // up
            {
                new_idx = idx + size_x_;
                if (!visited[new_idx])
                {
                    q.emplace(new_idx, val);
                    visited[new_idx] = true;
                }
            }
        }
    }
}
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(dwa_critics::GlobalTrajectoryAlignCritic, dwa_critics::BaseCritic)