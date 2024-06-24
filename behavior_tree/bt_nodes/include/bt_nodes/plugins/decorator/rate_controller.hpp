#ifndef RATE_CONTROLELR_HPP_
#define RATE_CONTROLELR_HPP_

#include <string>
#include <chrono>

#include "behaviortree_cpp/decorator_node.h"

namespace bt_nodes
{
class RateController : public BT::DecoratorNode
{
public:
    RateController(
        const std::string& name,
        const BT::NodeConfiguration& conf);
    static BT::PortsList providedPorts()
    {
        return {BT::InputPort<double>("hz", 10.0, "Rate")};
    }
private:
    BT::NodeStatus tick() override;
    std::chrono::time_point<std::chrono::high_resolution_clock> start_;
    double period_;
    bool first_time_{false};
};
}

#endif