#ifndef NAVIGATE_TO_POSE_NAVIGATOR_HPP_
#define NAVIGATE_TO_POSE_NAVIGATOR_HPP_

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "bt_navigators/bt_navigator.hpp"
#include "gnc_msgs/action/navigate_to_pose.hpp"

namespace bt_navigators
{
class NavigateToPoseNavigator : public BTNavigator<gnc_msgs::action::NavigateToPose>
{
public:
    using ActionT = gnc_msgs::action::NavigateToPose;

    NavigateToPoseNavigator();

    bool configure(rclcpp_lifecycle::LifecycleNode::WeakPtr parent) override;
    bool cleanup() override;

    std::string getDefaultBTFilepath(rclcpp_lifecycle::LifecycleNode::WeakPtr parent) override;
    std::string getName() override {return std::string("navigate_to_pose");};

protected:
    bool onGoalReceived(typename ActionT::Goal::ConstSharedPtr goal) override;
    void onLoop() override;
    void onPreempt(typename ActionT::Goal::ConstSharedPtr goal) override;
    void onCompletion(typename ActionT::Result::SharedPtr result,
        const bt_navigators::BTStatus final_bt_status) override;

    void onGoalPoseReceived(const geometry_msgs::msg::PoseStamped::SharedPtr pose);
    bool initializeGoalPose(ActionT::Goal::ConstSharedPtr goal);

    rclcpp::Time start_time_;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp_action::Client<ActionT>::SharedPtr self_client_;

    std::string goal_blackboard_id_;
    std::string path_blackboard_id_;
};
}

#endif