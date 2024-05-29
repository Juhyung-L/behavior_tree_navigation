#include "bt_nodes/plugins/action/follow_path_action.hpp"
#include "gnc_msgs/action/follow_path.hpp"

namespace bt_nodes
{
FollowPathAction::FollowPathAction(
    const std::string& xml_tag_name,
    const std::string& action_name,
    const BT::NodeConfiguration& conf)
: BTActionNode<gnc_msgs::action::FollowPath>(xml_tag_name, action_name, conf)
{}

void FollowPathAction::on_tick()
{
    getInput<nav_msgs::msg::Path>("path", goal_.path);
}

void FollowPathAction::on_wait_for_result(
    std::shared_ptr<const gnc_msgs::action::FollowPath::Feedback> /*feedback*/)
{
    // update goal if new path came in
    nav_msgs::msg::Path new_path;
    getInput<nav_msgs::msg::Path>("path", new_path);

    if (goal_.path != new_path)
    {
        goal_.path = new_path;
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
            return std::make_unique<bt_nodes::FollowPathAction>(
                name, "follow_path", config);
        };

    factory.registerBuilder<bt_nodes::FollowPathAction>("FollowPath", builder);
}