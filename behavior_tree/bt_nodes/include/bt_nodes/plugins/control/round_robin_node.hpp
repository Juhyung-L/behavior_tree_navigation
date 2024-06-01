#ifndef ROUND_ROBIN_NODE_HPP_
#define ROUND_ROBIN_NODE_HPP_

#include "behaviortree_cpp/control_node.h"

namespace bt_nodes
{
class RoundRobinNode : public BT::ControlNode
{
public:
    explicit RoundRobinNode(const std::string& name);
    RoundRobinNode(const std::string& name, const BT::NodeConfiguration& config);
    BT::NodeStatus tick() override;
    void halt() override;
    static BT::PortsList providedPorts() {return {};}
private:
    size_t current_child_idx_{0};
    size_t num_failed_children_{0};
};
}

#endif