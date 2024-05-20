#ifndef BT_NAVIGATOR_NODE_HPP_
#define BT_NAVIGATOR_NODE_HPP_

#include <string>
#include <memory>
#include <vector>

#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/create_timer_ros.h"
#include "pluginlib/class_loader.hpp"

#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/odometry_utils.hpp"
#include "bt_navigator/bt_navigator.hpp"

namespace bt_navigator
{
class BTNavigatorNode : public nav2_util::LifecycleNode
{
public:
    explicit BTNavigatorNode(rclcpp::NodeOptions options = rclcpp::NodeOptions());
    ~BTNavigatorNode();

protected:
    nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
    nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
    nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
    nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
    nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;
    
    pluginlib::ClassLoader<BTNavigatorBase> class_loader_;
    pluginlib::UniquePtr<BTNavigatorBase> navigator_;
    std::shared_ptr<nav2_util::OdomSmoother> odom_smoother_;
    std::shared_ptr<tf2_ros::Buffer> tf_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};
}

#endif