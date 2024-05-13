#ifndef GLOBAL_TRAJECTORY_ALIGN_HPP_
#define GLOBAL_TRAJECTORY_ALIGN_HPP_

#include <vector>
#include <queue>

#include "geometry_msgs/msg/pose2_d.hpp"

#include "nav_2d_msgs/msg/path2_d.hpp"
#include "dwa_critics/base_critic.hpp"

namespace dwa_critics
{
struct Node
{
    Node(unsigned int idx)
    : idx(idx)
    {}

    Node(unsigned int idx, unsigned char parent_val)
    : idx(idx)
    , parent_val(parent_val)
    {}

    unsigned int idx;
    unsigned char parent_val;
};
/**
 * @class GlobalTrajectoryAlignCritic
 * @brief A critic penalizes trajectories that do not align with the global trajectory
*/
class GlobalTrajectoryAlignCritic : public BaseCritic 
{
public:
    GlobalTrajectoryAlignCritic();
    /**
     * @param Trajectory Global Trajectory in map frame
    */
    void prepare(const nav_2d_msgs::msg::Path2D& global_traj, const geometry_msgs::msg::Pose2D& goal_pose) override;
    double scoreTrajectory(const nav_2d_msgs::msg::Path2D& local_traj) override;
    virtual ~GlobalTrajectoryAlignCritic() = default;

private:
    void on_initialize() override;
    void makeAlignmentMap(const nav_2d_msgs::msg::Path2D& Trajectory);
    std::vector<unsigned char> alignment_map_;
    unsigned int size_x_;
    unsigned int size_y_;
};
}

#endif