#ifndef PROGRESS_CHECKER_HPP_
#define PROGRESS_CHECKER_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"

namespace gnc_core
{
/**
 * @class ProgressChecker
 * @brief Base class for all progress checkers. All progress checkers implement the checker()
 * function, which determines if the robot is progressing towards a goal or not
*/
class ProgressChecker
{
public:
    using Ptr = std::shared_ptr<gnc_core::ProgressChecker>;

    virtual ~ProgressChecker() {}

    virtual void initialize(const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent,
        const std::string& plugin_name) = 0;

    virtual bool check(geometry_msgs::msg::PoseStamped & current_pose) = 0;
    virtual void reset() = 0;
};
}

#endif