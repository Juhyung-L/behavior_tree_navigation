#ifndef PLANNER_SERVER_HPP_
#define PLANNER_SERVER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/node_thread.hpp"
#include "nav2_util/simple_action_server.hpp"
#include "pluginlib/class_loader.hpp"

#include "gnc_core/planner.hpp"
#include "gnc_msgs/action/compute_path_to_pose.hpp"

namespace planner_server
{
class PlannerServer : public nav2_util::LifecycleNode
{
    using ActionServer = nav2_util::SimpleActionServer<gnc_msgs::action::ComputePathToPose>;
public:
    explicit PlannerServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~PlannerServer();
protected:
    nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State& state) override;
    nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State& state) override;
    nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& state) override;
    nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State& state) override;
    nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State& state) override;

    gnc_core::Planner::Ptr planner_;
    pluginlib::ClassLoader<gnc_core::Planner> planner_loader_;
    std::string planner_type_;
    double planner_frequency_;

    std::shared_ptr<tf2_ros::Buffer> tf_;
    rclcpp::Logger logger_;
    std::unique_ptr<ActionServer> action_server_;

    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
    std::unique_ptr<nav2_util::NodeThread> costmap_thread_;
    nav2_costmap_2d::Costmap2D* costmap_;

    rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

    void executePlanner();
};
}

#endif