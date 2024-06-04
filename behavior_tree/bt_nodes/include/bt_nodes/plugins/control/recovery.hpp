#ifndef RECOVERY_HPP_
#define RECOVERY_HPP_

#include <string>
#include "behaviortree_cpp/control_node.h"

namespace bt_nodes
{
class Recovery : public BT::ControlNode
{
public:
    Recovery(
        const std::string& name,
        const BT::NodeConfiguration& conf);

    ~Recovery() override = default;
    
    static BT::PortsList providedPorts()
    {
        return 
        {
            BT::InputPort<int>("number_of_retries", 1, "Number of retries")
        };
    }

private:
    unsigned int current_child_idx_;
    unsigned int number_of_retries_;
    unsigned int retry_count_;

protected:
    BT::NodeStatus tick() override;
    void halt() override;
};
}
#endif