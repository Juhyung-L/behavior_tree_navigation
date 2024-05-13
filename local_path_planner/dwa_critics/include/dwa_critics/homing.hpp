#ifndef DISTANCE_TO_GOAL_HPP_
#define DISTANCE_TO_GOAL_HPP_

#include "geometry_msgs/msg/pose2_d.hpp"

#include "nav_2d_msgs/msg/path2_d.hpp"
#include "dwa_critics/base_critic.hpp"

namespace dwa_critics
{
/**
 * @class HomingCritic
 * @brief If the goal pose is inside the local costmap, score local trajectories based on distance between goal pose and last pose of local trajectory
*/
class HomingCritic : public BaseCritic
{
public:
    HomingCritic();
    void prepare(const nav_2d_msgs::msg::Path2D& globa_traj, const geometry_msgs::msg::Pose2D& goal_pose) override;
    double scoreTrajectory(const nav_2d_msgs::msg::Path2D& local_traj) override;
    virtual ~HomingCritic() = default;
private:
    void on_initialize() override;
    geometry_msgs::msg::Pose2D goal_pose_;
    bool goal_inside_costmap_;
};
}

#endif