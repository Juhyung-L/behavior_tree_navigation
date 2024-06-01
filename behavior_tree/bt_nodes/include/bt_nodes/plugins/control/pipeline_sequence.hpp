#ifndef PIPELINE_SEQUENCE_HPP_
#define PIPELINE_SEQUENCE_HPP_

#include <string>

#include "behaviortree_cpp/control_node.h"

namespace bt_nodes
{
class PipelineSequence : public BT::ControlNode
{
public:
    explicit PipelineSequence(const std::string& name);
    PipelineSequence(const std::string& name, const BT::NodeConfiguration& config);
    void halt() override;
    static BT::PortsList providedPorts() {return {};}
protected:
    BT::NodeStatus tick() override;

    size_t last_child_ticked_ = 0;
};
}

#endif