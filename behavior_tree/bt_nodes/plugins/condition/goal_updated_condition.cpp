#include "bt_nodes/plugins/condition/goal_updated_condition.hpp"

namespace bt_nodes
{
GoalUpdatedCondition::GoalUpdatedCondition(
    const std::string& condition_name,
    const BT::NodeConfiguration& conf)
: BT::ConditionNode(condition_name, conf)
{}

BT::NodeStatus GoalUpdatedCondition::tick()
{
    if (status() == BT::NodeStatus::IDLE)
    {
        [[maybe_unused]] auto res = config().blackboard->get<geometry_msgs::msg::PoseStamped>("goal", prev_goal_);
        return BT::NodeStatus::FAILURE;
    }

    geometry_msgs::msg::PoseStamped current_goal;
    [[maybe_unused]] auto res = config().blackboard->get<geometry_msgs::msg::PoseStamped>("goal", current_goal);
    if (current_goal != prev_goal_)
    {
        prev_goal_ = current_goal;
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
}
}

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<bt_nodes::GoalUpdatedCondition>("GoalUpdated");
}