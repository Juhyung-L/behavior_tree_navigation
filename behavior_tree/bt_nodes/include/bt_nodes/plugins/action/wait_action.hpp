#ifndef WAIT_ACTION_HPP_
#define WAIT_ACTION_HPP_

#include <string>

#include "bt_nodes/bt_action_node.hpp"
#include  "gnc_msgs/action/wait.hpp"

namespace bt_nodes
{
class WaitAction : public BTActionNode<gnc_msgs::action::Wait>
{
public:
    WaitAction(
        const std::string& xml_tag_name,
        const std::string& action_name,
        const BT::NodeConfiguration& conf);

    void on_tick() override;

    static BT::PortsList providedPorts()
    {
        return providedBasicPorts({
            BT::InputPort<int>("wait_duration", 1, "Wait time")});
    }
};
}
#endif