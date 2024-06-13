#ifndef BEHAVIOR_SERVER_HPP_
#define BEHAVIOR_SERVER_HPP_

#include <string>
#include <vector>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "pluginlib/class_loader.hpp"
#include "nav2_costmap_2d/costmap_topic_collision_checker.hpp"
#include "nav2_costmap_2d/footprint_subscriber.hpp"
#include "nav2_costmap_2d/costmap_subscriber.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "gnc_core/behavior.hpp"

namespace behavior_server
{
class BehaviorServer : public nav2_util::LifecycleNode
{
public:
    explicit BehaviorServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~BehaviorServer(); 

protected:
    nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State& state) override;
    nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State& state) override;
    nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& state) override;
    nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State& state) override;
    nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State& state) override;

private:
    bool loadBehaviors();

    pluginlib::ClassLoader<gnc_core::Behavior> plugin_loader_;
    std::vector<pluginlib::UniquePtr<gnc_core::Behavior>> behaviors_;

    std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub_;
    std::unique_ptr<nav2_costmap_2d::FootprintSubscriber> footprint_sub_;
    std::shared_ptr<nav2_costmap_2d::CostmapTopicCollisionChecker> collision_checker_;

    std::shared_ptr<tf2_ros::Buffer> tf_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::Logger logger_{rclcpp::get_logger("behavior_server")};
};
}

#endif