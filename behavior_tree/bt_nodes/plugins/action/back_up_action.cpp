#include "bt_nodes/plugins/action/back_up_action.hpp"

namespace bt_nodes
{
BackUpAction::BackUpAction(
    const std::string& xml_tag_name,
    const std::string& action_name,
    const BT::NodeConfiguration& conf)
: BTActionNode<gnc_msgs::action::BackUp>(xml_tag_name, action_name, conf)
{
    double dist;
    getInput<double>("back_up_dist", dist);
    double speed;
    getInput<double>("back_up_speed", speed);
    double duration;
    getInput<double>("back_up_duration", duration);

    // Populate the input message
    goal_.back_up_distance = dist;
    goal_.back_up_speed = speed;
    goal_.duration = rclcpp::Duration::from_seconds(duration);
}

void BackUpAction::on_tick()
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
            return std::make_unique<bt_nodes::BackUpAction>(
                name, "back_up", config);
        };

    factory.registerBuilder<bt_nodes::BackUpAction>("BackUp", builder);
}