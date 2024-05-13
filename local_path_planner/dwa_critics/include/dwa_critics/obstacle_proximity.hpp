#ifndef OBSTACLE_PROXIMITY_HPP_
#define OBSTACLE_PROXIMITY_HPP_

#include "geometry_msgs/msg/pose2_d.hpp"

#include "nav_2d_msgs/msg/path2_d.hpp"
#include "dwa_critics/base_critic.hpp"

namespace dwa_critics
{
/**
 * @class ObstacleProximityCritic
 * @brief A critic that penalizes trajectories that are close to obstacles
*/
class ObstacleProximityCritic : public BaseCritic
{
public:
    ObstacleProximityCritic();
    void prepare(const nav_2d_msgs::msg::Path2D& globa_traj, const geometry_msgs::msg::Pose2D& goal_pose) override;
    double scoreTrajectory(const nav_2d_msgs::msg::Path2D& local_traj) override;
    virtual ~ObstacleProximityCritic() = default;
private:
    void on_initialize() override;
};
}
#endif