#include "bt_nodes/plugins/action/compute_path_to_pose_action.hpp"

namespace bt_nodes
{
ComputePathToPoseAction::ComputePathToPoseAction(
    const std::string& xml_tag_name,
    const std::string& action_name,
    const BT::NodeConfiguration& conf)
: BTActionNode<Action>(xml_tag_name, action_name, conf)
{}

void ComputePathToPoseAction::on_tick()
{
    getInput<geometry_msgs::msg::PoseStamped>("goal", goal_.goal);
}

BT::NodeStatus ComputePathToPoseAction::on_success()
{
    setOutput<nav_msgs::msg::Path>("path", result_.result->path);
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus ComputePathToPoseAction::on_aborted()
{
    nav_msgs::msg::Path empty_path;
    setOutput<nav_msgs::msg::Path>("path", empty_path);
    return BT::NodeStatus::FAILURE;
}

BT::NodeStatus ComputePathToPoseAction::on_cancelled()
{
    nav_msgs::msg::Path empty_path;
    setOutput<nav_msgs::msg::Path>("path", empty_path);
    return BT::NodeStatus::SUCCESS;
}

void ComputePathToPoseAction::halt()
{
    nav_msgs::msg::Path empty_path;
    setOutput<nav_msgs::msg::Path>("path", empty_path);
    BTActionNode::halt();
}

void ComputePathToPoseAction::on_wait_for_result(
    std::shared_ptr<const gnc_msgs::action::ComputePathToPose::Feedback> /*feedback*/)
{
    geometry_msgs::msg::PoseStamped new_goal;    
    getInput<geometry_msgs::msg::PoseStamped>("goal", new_goal);
    if (goal_.goal != new_goal)
    {
        goal_.goal = new_goal;
        goal_updated_ = true;
    }
}
}

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    BT::NodeBuilder builder =
        [](const std::string& name, const BT::NodeConfiguration& config)
        {
            return std::make_unique<bt_nodes::ComputePathToPoseAction>(
                name, "compute_path_to_pose", config);
        };
    factory.registerBuilder<bt_nodes::ComputePathToPoseAction>("ComputePathToPose", builder);
}