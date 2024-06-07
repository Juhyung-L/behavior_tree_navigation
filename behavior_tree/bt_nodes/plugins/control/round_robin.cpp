#include "bt_nodes/plugins/control/round_robin.hpp"

namespace bt_nodes
{
RoundRobin::RoundRobin(const std::string& name)
: BT::ControlNode::ControlNode(name, {})
{}

RoundRobin::RoundRobin(
  const std::string& name,
  const BT::NodeConfiguration& config)
: BT::ControlNode(name, config)
{}

BT::NodeStatus RoundRobin::tick()
{
    const size_t num_children = children_nodes_.size();

    setStatus(BT::NodeStatus::RUNNING);
    size_t num_skipped_children = 0;

    while (num_failed_children_ + num_skipped_children < num_children) 
    {
        TreeNode* child_node = children_nodes_[current_child_idx_];
        const BT::NodeStatus child_status = child_node->executeTick();

        if (child_status != BT::NodeStatus::RUNNING) 
        {
            // increment index and wrap around to the first child
            if (++current_child_idx_ == num_children) 
            {
                current_child_idx_ = 0;
            }
        }

        switch (child_status) 
        {
            case BT::NodeStatus::SUCCESS:
            {
                num_failed_children_ = 0;
                ControlNode::haltChildren();
                return BT::NodeStatus::SUCCESS;
            }
            case BT::NodeStatus::FAILURE:
            {
                num_failed_children_++;
                break;
            }
            case BT::NodeStatus::SKIPPED:
            {
                num_skipped_children++;
                break;
            }
            case BT::NodeStatus::RUNNING:
                return BT::NodeStatus::RUNNING;
            default:
                throw BT::LogicError(
                    "Child node of [" + name() + "] returned IDLE when it shouldn't\n"
                    "Child node: [" + children_nodes_[current_child_idx_]->name());
        }
    }
    const bool all_skipped = (num_skipped_children == num_children);
    halt();
    // if all the children were skipped, this node is considered skipped
    return all_skipped ? BT::NodeStatus::SKIPPED : BT::NodeStatus::FAILURE;
}

void RoundRobin::halt()
{
    ControlNode::halt();
    current_child_idx_ = 0;
    num_failed_children_ = 0;
}
}

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<bt_nodes::RoundRobin>("RoundRobin");
}