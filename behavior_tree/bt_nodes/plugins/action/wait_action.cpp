#include "bt_nodes/plugins/action/wait_action.hpp"

namespace bt_nodes
{
WaitAction::WaitAction(
    const std::string& xml_tag_name,
    const std::string& action_name,
    const BT::NodeConfiguration& conf)
: BTActionNode<gnc_msgs::action::Wait>(xml_tag_name, action_name, conf)
{
    int duration;
    getInput<int>("wait_duration", duration);
    if (duration <= 0) 
    {
        RCLCPP_WARN(node_->get_logger(), 
            "Wait duration is negative or zero "
            "(%i). Setting to positive.", duration);
        duration *= -1;
    }

    goal_.duration = rclcpp::Duration::from_seconds(duration);
}

void WaitAction::on_tick()
{
    increment_recovery_count();
}
}

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    BT::NodeBuilder builder =
        [](const std::string& name, const BT::NodeConfiguration& config)
        {
            return std::make_unique<bt_nodes::WaitAction>(name, "wait", config);
        };

    factory.registerBuilder<bt_nodes::WaitAction>("Wait", builder);
}