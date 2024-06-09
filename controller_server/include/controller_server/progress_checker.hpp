#ifndef PROGRESS_CHECKER_HPP_
#define PROGRESS_CHECKER_HPP_

#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"

namespace controller_server
{
class ProgressChecker
{
public:
    void initialize(const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent);
    bool isProgressed(geometry_msgs::msg::Pose& cur_pose);
    void reset();
private:
    geometry_msgs::msg::Pose baseline_pose_;
    bool baseline_set_{false};
    rclcpp::Time baseline_time_;

    rclcpp::Clock::SharedPtr clock_;

    // the robot must move minimum_distance_ in less than movement_time_allowance_ time
    // to be considered as progressed
    double minimum_distance_;
    rclcpp::Duration movement_time_allowance_{rclcpp::Duration::from_seconds(0.0)};

    void setBaseline(const geometry_msgs::msg::Pose& pose);
    bool movedEnough(const geometry_msgs::msg::Pose& cur_pose);
};
}

#endif