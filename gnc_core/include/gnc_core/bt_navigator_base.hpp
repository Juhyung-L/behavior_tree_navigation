#ifndef BT_NAVIGATOR_BASE_HPP_
#define BT_NAVIGATOR_BASE_HPP_

#include <string>
#include <memory>

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_util/odometry_utils.hpp"
#include "tf2_ros/buffer.h"

namespace gnc_core
{
/**
 * plugin architecture:
 * BTNavigator<ActionT> is a template class and therefore can have multiple implementations (ActionT is a ROS action)
 * We want to be able have a pointer to the base class and point it to a plugin that derives from the base class.
 * But, we can't do this with BTNavigator since it is a template class. So, we make another base class that 
 * BTNavigator derives from: BtNavigatorBase. Because we will only have a pointer to BTNavigatorBase, we need to 
 * make sure the class has all the necessary public functions to operate any plugin that derives from it.
*/
class BTNavigatorBase
{
public:
    BTNavigatorBase() = default;
    virtual ~BTNavigatorBase() = default;
    virtual bool on_configure(rclcpp_lifecycle::LifecycleNode::WeakPtr parent,
        std::shared_ptr<nav2_util::OdomSmoother> odom_smoother,
        const std::vector<std::string> & plugin_lib_names,
        std::shared_ptr<tf2_ros::Buffer> tf,
        std::string robot_frame,
        std::string global_frame,
        double transform_tolerance) = 0;
    virtual bool on_activate() = 0;
    virtual bool on_deactivate() = 0;
    virtual bool on_cleanup() = 0;

protected:
    std::string plugin_name_;
};
}

#endif