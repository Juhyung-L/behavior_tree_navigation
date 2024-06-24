#include "bt_nodes/plugins/control/recovery.hpp"

namespace bt_nodes
{
Recovery::Recovery(
    const std::string& name,
    const BT::NodeConfiguration& conf)
: BT::ControlNode::ControlNode(name, conf)
, current_child_idx_(0)
, number_of_retries_(1)
, retry_count_(0)
{}

/**
 * This node can only have 2 children.
 * If the first child returns SUCCESS, just return SUCCESS.
 * If the first child returns FAILURE, tick the second (recovery) child.
 * - if the second child returns SUCCESS, return RUNNING and in the next tick, 
 *   the first child is ticked again.
 * - if the second child returns FAILURE, return FAILURE (recovery failed).
 * Before ticking the second node, check if the max number of recoveries
 * were already attempted, in which case return FAILURE.
*/
BT::NodeStatus Recovery::tick()
{
    // number_of_retries_ = max number of recovery attempts
    getInput<unsigned int>("number_of_retries", number_of_retries_);
    size_t num_children = children_nodes_.size();

    if (num_children != 2)
    {
        throw std::runtime_error(
            "Recovery Node [" + name() + "] must only have 2 children.\n"
            "It has " + std::to_string(num_children) + " children");
    }

    setStatus(BT::NodeStatus::RUNNING);

    while (current_child_idx_ < num_children && retry_count_ <= number_of_retries_) 
    {
        TreeNode* child_node = children_nodes_[current_child_idx_];
        const BT::NodeStatus child_status = child_node->executeTick();

        if (current_child_idx_ == 0) 
        {
            switch (child_status) 
            {
                case BT::NodeStatus::SKIPPED:
                    // if first child is skipped, the entire branch is considered skipped
                    halt();
                    return BT::NodeStatus::SKIPPED;
                case BT::NodeStatus::SUCCESS:
                    // reset node and return success when first child returns success
                    halt();
                    return BT::NodeStatus::SUCCESS;
                case BT::NodeStatus::RUNNING:
                    return BT::NodeStatus::RUNNING;
                case BT::NodeStatus::FAILURE:
                {
                    if (retry_count_ < number_of_retries_) 
                    {
                        // halt first child and tick second child in next iteration
                        ControlNode::haltChild(0);
                        current_child_idx_++;
                        return BT::NodeStatus::RUNNING;
                    } 
                    else 
                    {
                        // reset node and return failure when max retries has been exceeded
                        halt();
                        return BT::NodeStatus::FAILURE;
                    }
                }
                default:
                    throw BT::LogicError(
                        "Child node of [" + name() + "] returned IDLE when it shouldn't\n"
                        "Child node: [" + children_nodes_[current_child_idx_]->name());
            }
        } 
        else if (current_child_idx_ == 1) 
        {
            switch (child_status) 
            {
                case BT::NodeStatus::SKIPPED:
                {
                    /**
                     * If we skip the recovery (maybe a precondition fails), then we should assume 
                     * that no recovery is possible. For this reason, we should return FAILURE and
                     * reset the index. This does not count as a retry.
                    */
                    current_child_idx_ = 0;
                    ControlNode::haltChild(1);
                    return BT::NodeStatus::FAILURE;
                }
                case BT::NodeStatus::RUNNING:
                    return child_status;
                case BT::NodeStatus::SUCCESS:
                {
                    // halt second child, increment recovery count, and tick first child in next iteration
                    ControlNode::haltChild(1);
                    retry_count_++;
                    current_child_idx_ = 0;
                    return BT::NodeStatus::RUNNING;
                }
                case BT::NodeStatus::FAILURE:
                    // reset node and return failure if second child fails
                    halt();
                    return BT::NodeStatus::FAILURE;
                default:
                    throw BT::LogicError(
                        "Child node of [" + name() + "] returned IDLE when it shouldn't\n"
                        "Child node: [" + children_nodes_[current_child_idx_]->name());
            }
        }
    }

    // this should never be reached since all of the case statements have a return (or throws error) statement
    halt();
    return BT::NodeStatus::FAILURE;
}

void Recovery::halt()
{
    ControlNode::halt();
    retry_count_ = 0;
    current_child_idx_ = 0;
}
}

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<bt_nodes::Recovery>("Recovery");
}
