#include "bt_nodes/plugins/decorator/rate_controller.hpp"

namespace bt_nodes
{
RateController::RateController(
    const std::string& name,
    const BT::NodeConfiguration& conf)
: BT::DecoratorNode(name, conf)
{
    double hz = 1.0;
    getInput<double>("hz", hz);
    period_ = 1.0 / hz;
}

/**
 * This node can only have 1 child. It ticks the child at the first tick. 
 * In the subsequent ticks does ths following:
 * - If the child returns RUNNING, tick it again.
 * - If the child returns SUCCESS before the period has expired, tick it (restarting the node) again just
 * after the period expires.
 */

BT::NodeStatus RateController::tick()
{
    if (status() == BT::NodeStatus::IDLE)
    {
        start_ = std::chrono::high_resolution_clock::now();
        first_time_ = true;
    }

    setStatus(BT::NodeStatus::RUNNING);

    auto now = std::chrono::high_resolution_clock::now();
    auto elapsed = now - start_;

    auto seconds = std::chrono::duration_cast<std::chrono::duration<double>>(elapsed);

    if (first_time_ || child_node_->status() == BT::NodeStatus::RUNNING || seconds.count() >= period_)
    {
        first_time_ = false;
        const BT::NodeStatus status = child_node_->executeTick();

        switch (status)
        {
            case BT::NodeStatus::RUNNING:
                return BT::NodeStatus::RUNNING;
            case BT::NodeStatus::SUCCESS:
                start_ = std::chrono::high_resolution_clock::now();
                return BT::NodeStatus::SUCCESS;
            case BT::NodeStatus::FAILURE:
                break;
            default:
                return BT::NodeStatus::FAILURE;
        }
    }
    return status();
}
}

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<bt_nodes::RateController>("RateController");
}