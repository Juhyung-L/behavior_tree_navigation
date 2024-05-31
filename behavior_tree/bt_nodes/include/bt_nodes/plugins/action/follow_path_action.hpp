#ifndef FOLLOW_PATH_ACTION_HPP_
#define FOLLOW_PATH_ACTION_HPP_

#include "bt_nodes/bt_action_node.hpp"
#include "gnc_msgs/action/follow_path.hpp"

namespace bt_nodes
{
class FollowPathAction : public BTActionNode<gnc_msgs::action::FollowPath>
{
public:
    FollowPathAction(
        const std::string& xml_tag_name,
        const std::string& action_name,
        const BT::NodeConfiguration& conf);

    void on_tick() override;
    BT::NodeStatus on_success() override;
    BT::NodeStatus on_aborted() override;
    BT::NodeStatus on_cancelled() override;

    void on_wait_for_result(
        std::shared_ptr<const gnc_msgs::action::FollowPath::Feedback> feedback) override;

    static BT::PortsList providedPorts()
    {
        return providedBasicPorts({
            BT::InputPort<nav_msgs::msg::Path>("path", "Path to follow")});
    }
};
}

#endif