#ifndef BACK_UP_ACTION_HPP_
#define BACK_UP_ACTION_HPP_

#include <string>

#include "bt_nodes/bt_action_node.hpp"
#include  "gnc_msgs/action/back_up.hpp"

namespace bt_nodes
{
class BackUpAction : public BTActionNode<gnc_msgs::action::BackUp>
{
public:
    BackUpAction(
        const std::string& xml_tag_name,
        const std::string& action_name,
        const BT::NodeConfiguration& conf);

    void on_tick() override;

    static BT::PortsList providedPorts()
    {
        return providedBasicPorts({
            BT::InputPort<double>("back_up_dist", 0.3, "Distance to backup"),
            BT::InputPort<double>("back_up_speed", 0.3, "Speed at which to backup"),
            BT::InputPort<double>("back_up_duration", 5.0, "Allowed time for reversing")});
    }
};
}
#endif