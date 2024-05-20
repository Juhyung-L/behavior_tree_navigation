#include <limits>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include "nav2_util/robot_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/node_utils.hpp"
#include "bt_navigator/navigate_to_pose_navigator.hpp"

/**
 * ROS parameters declared
 * - goal_blackboard_id
 * - path_blackboard_id
 * - default_bt_xml_filename
*/

namespace bt_navigator
{
// called in on_configure()
bool NavigateToPoseNavigator::configure(rclcpp_lifecycle::LifecycleNode::WeakPtr parent)
{
    auto node = parent.lock();
    // when delcaring parameters outside of the constructor, 
    // you need to check if the parameter already exists before declaring
    nav2_util::declare_parameter_if_not_declared(node, "goal_blackboard_id", rclcpp::ParameterValue(std::string("goal")));
    goal_blackboard_id_ = node->get_parameter("goal_blackboard_id").as_string();
    nav2_util::declare_parameter_if_not_declared(node, "path_blackboard_id", rclcpp::ParameterValue(std::string("path")));
    path_blackboard_id_ = node->get_parameter("path_blackboard_id").as_string();

    self_client_ = rclcpp_action::create_client<ActionT>(node, getName());

    goal_sub_ = node->create_subscription<geometry_msgs::msg::PoseStamped>(
        "goal_pose",
        rclcpp::SystemDefaultsQoS(),
        std::bind(&NavigateToPoseNavigator::onGoalPoseReceived, this, std::placeholders::_1));
    
    start_time_ = rclcpp::Time(0);
    return true;
}

std::string
NavigateToPoseNavigator::getDefaultBTFilepath(rclcpp_lifecycle::LifecycleNode::WeakPtr parent)
{
    auto node = parent.lock();
    std::string default_bt_xml_filename;
    if (node->has_parameter("default_bt_xml_filename"))
    {
        std::string default_bt_xml_filename = 
            ament_index_cpp::get_package_share_directory("bt_navigator") + 
            "/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml";
        node->declare_parameter("default_bt_xml_filename", default_bt_xml_filename);
    }
    default_bt_xml_filename = node->get_parameter("default_bt_xml_filename").as_string();
    return default_bt_xml_filename;
}

// called in on_cleanup()
bool NavigateToPoseNavigator::cleanup()
{
    self_client_.reset();
    goal_sub_.reset();
    return true;
}

void NavigateToPoseNavigator::onGoalPoseReceived(const geometry_msgs::msg::PoseStamped::SharedPtr pose)
{
    ActionT::Goal goal;
    goal.pose = *pose;
    self_client_->async_send_goal(goal);
}

// callbacks sent to bt_action_server_
bool NavigateToPoseNavigator::onGoalReceived(ActionT::Goal::ConstSharedPtr goal)
{
    // bt_action_server already has a tree loaded from the default xml filename after activate() is called
    // if the default file is the same as goal->behavior_tree, it won't do anything
    // if default file is not the same as goal->behavior_tree, it will load a new tree
    // it will return false if something goes wrong while loading the tree from file
    if (!bt_action_server_->loadBehaviorTree(goal->behavior_tree))
    {
        return false;
    }
    return initializeGoalPose(goal);
}

void
NavigateToPoseNavigator::onCompletion(
    typename ActionT::Result::SharedPtr /*result*/,
    const bt_interface::BTStatus /*final_bt_status*/)
{}

void NavigateToPoseNavigator::onLoop()
{
    // feedback:
    // - current pose (current_pose)
    // - time elapsed since start of task (navigation_time)
    // - estimated distance remaining to goal (distance_remaining)
    // - estimated time remaining (estimated_time_remaining)
    // - number of recoveries performed so far (number_of_recoveries)
    auto feedback_msg = std::make_shared<ActionT::Feedback>();

    geometry_msgs::msg::PoseStamped cur_pose;
    if (!nav2_util::getCurrentPose(cur_pose,
        *tf_, global_frame_, robot_frame_, transform_tolerance_))
    {
        return;
    }
    auto blackboard = bt_action_server_->getBlackboard();

    // populate in feedback_msg if possible:
    // - distance_remaining
    // - estimated_time_remaining
    nav_msgs::msg::Path cur_path;
    if (blackboard->get<nav_msgs::msg::Path>(path_blackboard_id_, cur_path) &&
        !cur_path.poses.empty())
    {
        // find closest pose on the path to robot's current pose
        size_t closest_pose_idx = 0;
        double cur_min_dist = std::numeric_limits<double>::max();
        for (size_t i=0; i<cur_path.poses.size(); ++i)
        {
            double cur_dist = nav2_util::geometry_utils::euclidean_distance(cur_pose, cur_path.poses[i]);
            if (cur_dist < cur_min_dist)
            {
                cur_min_dist = cur_dist;
                closest_pose_idx = i;
            }
        }

        double distance_remaining = nav2_util::geometry_utils::calculate_path_length(
            cur_path, closest_pose_idx);
        rclcpp::Duration estimated_time_remaining = rclcpp::Duration::from_seconds(0.0);
        geometry_msgs::msg::Twist cur_vel = odom_smoother_->getTwist();
        double abs_cur_speed = std::abs(std::hypot(cur_vel.linear.x, cur_vel.linear.y));
        if (abs_cur_speed > 0.01)
        {
            estimated_time_remaining = rclcpp::Duration::from_seconds(distance_remaining / abs_cur_speed);
        }
        feedback_msg->distance_remaining = distance_remaining;
        feedback_msg->estimated_time_remaining = estimated_time_remaining;
    }

    // populate the rest of feedback
    int recovery_count = 0;
    [[maybe_unused]] auto _ = blackboard->get<int>("number_recoveries", recovery_count);
    feedback_msg->number_of_recoveries = recovery_count;
    feedback_msg->current_pose = cur_pose;
    feedback_msg->navigation_time = clock_->now() - start_time_;

    bt_action_server_->publishFeedback(feedback_msg);
}

void NavigateToPoseNavigator::onPreempt(typename ActionT::Goal::ConstSharedPtr goal)
{
    if (goal->behavior_tree == bt_action_server_->getCurrentBTFilename())
    {
        if (!initializeGoalPose(bt_action_server_->acceptPendingGoal()))
        {
            RCLCPP_WARN(logger_,
                "Preemption request was rejected since the goal pose could not be "
                "transformed. For now, continuing to track the last goal until completion.");
            bt_action_server_->terminatePendingGoal();
        }
        else
        {
            RCLCPP_WARN(logger_, "Preempting with a new behavior tree is not allowed");
            bt_action_server_->terminatePendingGoal();
        }
    }
}

bool NavigateToPoseNavigator::initializeGoalPose(ActionT::Goal::ConstSharedPtr goal)
{
    geometry_msgs::msg::PoseStamped cur_pose;
    if (!nav2_util::getCurrentPose(cur_pose,
        *tf_, global_frame_, robot_frame_, transform_tolerance_))
    {
        return false;
    }

    geometry_msgs::msg::PoseStamped goal_pose;
    if (!nav2_util::transformPoseInTargetFrame(
        goal->pose, goal_pose, *tf_, global_frame_, transform_tolerance_))
    {
        return false;
    }

    RCLCPP_INFO(logger_, 
        "Begin navigating from current location (%.2f, %.2f) to (%.2f, %.2f)",
        cur_pose.pose.position.x, cur_pose.pose.position.y,
        goal_pose.pose.position.x, goal_pose.pose.position.y);

    start_time_ = clock_->now();
    // set recovery count to 0 and upload goal pose to blackboard
    auto blackboard = bt_action_server_->getBlackboard();
    blackboard->set<int>("number_recoveries", 0);
    blackboard->set<geometry_msgs::msg::PoseStamped>(goal_blackboard_id_, goal_pose);
    return true;
}
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(bt_navigator::NavigateToPoseNavigator, bt_navigator::BTNavigatorBase);