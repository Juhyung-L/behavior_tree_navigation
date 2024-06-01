#include <sstream>

#include "bt_nodes/plugins/control/pipeline_sequence.hpp"

namespace bt_nodes
{
PipelineSequence::PipelineSequence(const std::string& name)
: BT::ControlNode(name, {})
{}

PipelineSequence::PipelineSequence(
    const std::string& name,
    const BT::NodeConfiguration& config)
: BT::ControlNode(name, config)
{}

// this node ticks the children in sequence
// if any child returns FAILURE, reset all children's state to IDLE (haltChildren() does this) and return SUCCESS
// if a child returns SUCCESS, just continue onto the next child
// - if the last child returns SUCCESS, reset all children (with haltChildren()) and return SUCCESS
// if a child returns RUNNING, don't tick the next child and return RUNNING
// if all children returned SKIPPED, return SKIPPED
BT::NodeStatus PipelineSequence::tick()
{
    unsigned skipped_count = 0;
    for (size_t i=0; i<children_nodes_.size(); ++i) 
    {
        auto status = children_nodes_[i]->executeTick();
        switch (status) 
        {
            case BT::NodeStatus::FAILURE:
                ControlNode::haltChildren();
                last_child_ticked_ = 0;
                return status;
            case BT::NodeStatus::SKIPPED:
                skipped_count++;
                break;
            case BT::NodeStatus::SUCCESS:
                break;
            case BT::NodeStatus::RUNNING:
                if (i >= last_child_ticked_) 
                {
                    last_child_ticked_ = i;
                    return status;
                }
                break;
            default:
                std::stringstream error_msg;
                error_msg << "Invalid node status. Received status " << status <<
                    "from child " << children_nodes_[i]->name();
                throw std::runtime_error(error_msg.str());
        }
    }
    // wrap up
    ControlNode::haltChildren();
    last_child_ticked_ = 0;
    
    // return SKIPPED if all children were skipped
    if (skipped_count == children_nodes_.size()) 
    {
        return BT::NodeStatus::SKIPPED;
    }
    return BT::NodeStatus::SUCCESS;
}

void PipelineSequence::halt()
{
    BT::ControlNode::halt();
    last_child_ticked_ = 0;
}
}

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<bt_nodes::PipelineSequence>("PipelineSequence");
}
