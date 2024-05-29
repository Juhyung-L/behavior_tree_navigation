#ifndef CONTROLLER_SERVER_HPP_
#define CONTROLLER_SERVER_HPP_

#include <memory>
#include <string>

#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/simple_action_server.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/node_thread.hpp"
#include "nav2_util/odometry_utils.hpp"
#include "tf2_ros/buffer.h"
#include "nav2_util/odometry_utils.hpp"

#include "gnc_core/contoller.hpp"
#include "gnc_msgs/action/follow_path.hpp"
#include "controller_server/goal_checker.hpp"
#include "controller_server/progress_checker.hpp"

namespace controller_server
{
class ControllerServer : public nav2_util::LifecycleNode
{
public:
    explicit ControllerServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    ~ControllerServer();
protected:
    nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State& state) override;
    nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State& state) override;
    nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& state) override;
    nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State& state) override;
    nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State& state) override;

private:
    using Action = gnc_msgs::action::FollowPath;
    using ActionServer = nav2_util::SimpleActionServer<Action>;

    std::unique_ptr<ActionServer> action_server_;

    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
    std::unique_ptr<nav2_util::NodeThread> costmap_thread_;

    gnc_core::Controller::Ptr controller_;
    pluginlib::ClassLoader<gnc_core::Controller> controller_loader_;
    std::string controller_type_;
    double controller_frequency_;

    std::unique_ptr<GoalChecker> goal_checker_;
    std::unique_ptr<ProgressChecker> progress_checker_;

    std::unique_ptr<nav2_util::OdomSmoother> odom_smoother_;
    std::string odom_topic_;

    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

    rclcpp::Logger logger_;

    void executeController();
    void publishFeedback(
        const geometry_msgs::msg::Pose& current_pose,
        const geometry_msgs::msg::Twist& current_velocity,
        const nav_msgs::msg::Path& global_path);
};
}

#endif