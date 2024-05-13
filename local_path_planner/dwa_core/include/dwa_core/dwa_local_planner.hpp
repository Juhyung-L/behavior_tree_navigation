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
#include "nav2_util/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "dwa_core/xytheta_velocity_iterator.hpp"
#include "dwa_core/kinematics_parameters.hpp"
#include "nav_2d_msgs/msg/path2_d.hpp"
#include "nav_2d_msgs/msg/dwa_trajectories.hpp"
#include "dwa_critics/base_critic.hpp"

constexpr double EPSILON = 1e-05;

namespace dwa_core
{
/**
 * @class DWALocalPlanner
 * @brief A path planner that uses the global trajectory as a guide to find an optimal local trajectory.
 * Samples velocity to generate local trajectories and scores them.
*/

class DWALocalPlanner : public nav2_util::LifecycleNode
{
public:
    explicit DWALocalPlanner(rclcpp::NodeOptions node_options = rclcpp::NodeOptions());
    ~DWALocalPlanner();

protected:
    nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
    nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
    nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
    nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
    nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
    KinematicsParameters::SharedPtr kp_;
    XYThetaVelocityIterator vel_it_;

    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
    std::unique_ptr<nav2_util::NodeThread> costmap_thread_;
    nav2_costmap_2d::Costmap2D* costmap_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    nav_msgs::msg::Odometry odom_;
    std::mutex odom_mtx_;
    std::string odom_topic_;

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr global_traj_sub_;
    nav_msgs::msg::Path global_traj_;
    std::mutex global_traj_mtx_;
    bool global_traj_set_;

    rclcpp_lifecycle::LifecyclePublisher<nav_2d_msgs::msg::DWATrajectories>::SharedPtr trajs_pub_;

    rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr global_traj_pub_;

    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

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

    rclcpp::Logger logger_;
    rclcpp::TimerBase::SharedPtr timer_;

    void computeVelocityCommand();
    nav_2d_msgs::msg::Path2D generateTrajectory(
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
    void odomCB(const nav_msgs::msg::Odometry& odom)
    {
        std::lock_guard<std::mutex> lock(odom_mtx_);
        odom_ = odom;
    }
    void globalTrajCB(const nav_msgs::msg::Path& global_traj)
    {
        std::lock_guard<std::mutex> lock(global_traj_mtx_);
        global_traj_ = global_traj;
        global_traj_set_ = true;
    }
    bool loadCritics();

    /**
     * @brief The global_traj_ variable gets updated everytime this node receives a global trajectory
     * from the global path planner. This function uses the global trajectory to generate an adjusted global trajectory by:
     * 1. cuting the global trajectory where the local costmap ends
     * 2. filling in poses in between consecutive poses to match local costmap's resolution
    */
    nav_2d_msgs::msg::Path2D prepareGlobalTrajectory(nav_2d_msgs::msg::Path2D in_traj);
};
}

#endif