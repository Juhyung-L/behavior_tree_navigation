#ifndef BASE_CRITIC_HPP_
#define BASE_CRITIC_HPP_

#include <memory>
#include <string>

#include "geometry_msgs/msg/pose2_d.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_util/lifecycle_node.hpp"

#include "nav_2d_msgs/msg/path2_d.hpp"

namespace dwa_critics
{
    /**
     * @class BaseCritic
     * @brief Abstract class that all critics inherit from
    */
class BaseCritic
{
public:
    using Ptr = std::shared_ptr<BaseCritic>;
    virtual ~BaseCritic() = default;
    void initialize(
        nav2_util::LifecycleNode::SharedPtr nh,
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
    {
        nh_ = nh;
        costmap_ros_ = costmap_ros;
        costmap_ = costmap_ros->getCostmap();
        on_initialize();
    }

    virtual void reset() {}
    /**
     * @param global_traj Global trajectory inside of the local costmap
     * @param goal_pose Final pose on the global trajectory (can be outside of the local costmap)
     * @brief The parameters provide exhaustive information about the global trajectory
     * and the derived classes might not require all of them
    */
    virtual void prepare(const nav_2d_msgs::msg::Path2D& global_traj, const geometry_msgs::msg::Pose2D& goal_pose) = 0;
    virtual double scoreTrajectory(const nav_2d_msgs::msg::Path2D& local_traj) = 0;
    double weight_;
    bool invert_score_;

protected:
    virtual void on_initialize() {}
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
    nav2_costmap_2d::Costmap2D* costmap_;
    nav2_util::LifecycleNode::SharedPtr nh_;
    std::string name_;
};
}

#endif