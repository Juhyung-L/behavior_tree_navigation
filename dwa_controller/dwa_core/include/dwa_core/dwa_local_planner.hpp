#ifndef DWA_LOCAL_PLANNER_HPP_
#define DWA_LOCAL_PLANNER_HPP_

#include <memory>
#include <string>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_util/node_thread.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "dwa_core/xytheta_velocity_iterator.hpp"
#include "dwa_core/kinematics_parameters.hpp"
#include "nav_2d_msgs/msg/path2_d.hpp"
#include "nav_2d_msgs/msg/dwa_trajectories.hpp"
#include "dwa_critics/base_critic.hpp"
#include "gnc_core/contoller.hpp"

constexpr double EPSILON = 1e-05;

namespace dwa_core
{
/**
 * @class DWALocalPlanner
 * @brief A path planner that uses the global path as a guide to find an optimal local path.
 * Samples velocity to generate local paths and scores them. 
 * This controller will be provided:
 * - robot's current pose
 * - robot's current velocity
 * - costmap
 * - global path
 * This controller will output:
 * - velocity commands
*/
class DWALocalPlanner : public gnc_core::Controller
{
public:
    explicit DWALocalPlanner();
    ~DWALocalPlanner();

protected:
    void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent,
        std::shared_ptr<tf2_ros::Buffer> tf,
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
    void activate() override;
    void deactivate() override;
    void cleanup() override;

    geometry_msgs::msg::Twist computeVelocityCommand(
        const geometry_msgs::msg::Pose& current_pose,
        const geometry_msgs::msg::Twist& current_velocity,
        const nav_msgs::msg::Path& global_path) override;
private:
    KinematicsParameters::SharedPtr kp_;
    XYThetaVelocityIterator vel_it_;

    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
    nav2_costmap_2d::Costmap2D* costmap_;

    bool global_path_set_{false};
    nav_msgs::msg::Path global_path_;

    rclcpp_lifecycle::LifecyclePublisher<nav_2d_msgs::msg::DWATrajectories>::SharedPtr paths_pub_;
    rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr adjusted_global_path_pub_;

    pluginlib::ClassLoader<dwa_critics::BaseCritic> critic_loader_;
    std::vector<dwa_critics::BaseCritic::Ptr> critics_;
    std::vector<std::string> critic_names_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    int prev_num_vel_samples_;
    double sim_time_;
    double time_granularity_;
    int steps_;
    bool debug_;
    std::string costmap_frame_;
    std::string plugin_name_;

    rclcpp::Logger logger_{rclcpp::get_logger("DWALocalPlanner")};

    nav_2d_msgs::msg::Path2D generatePath(
        const geometry_msgs::msg::Pose2D& start_pose,
        const nav_2d_msgs::msg::Twist2D& current_vel,
        const nav_2d_msgs::msg::Twist2D& target_vel);
    nav_2d_msgs::msg::Twist2D computeVelocity(
        const nav_2d_msgs::msg::Twist2D& current_vel, 
        const nav_2d_msgs::msg::Twist2D& target_vel,
        double dt);
    geometry_msgs::msg::Pose2D computePose(
        const geometry_msgs::msg::Pose2D& start_pose,
        const nav_2d_msgs::msg::Twist2D& vel,
        double dt);
    double projectVelocity(double current_vel, double acc, double decel, double dt, double target_vel);
    bool loadCritics(const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent);

    /**
     * @briefThis function uses the global path to generate an adjusted global path by:
     * 1. cuting the global path where the local costmap ends
     * 2. filling in poses in between consecutive poses to match local costmap's resolution
    */
    nav_2d_msgs::msg::Path2D prepareGlobalPath(nav_2d_msgs::msg::Path2D in_path);
};
}

#endif