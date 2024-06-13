#include "behaviors/plugins/wait.hpp"

namespace behaviors
{
Wait::Wait()
: TimedBehavior<Action>()
{
    behavior_name_ = "Wait";
    action_name_ = "wait";
}

ResultStatus Wait::onRun(const std::shared_ptr<const Action::Goal> command)
{
    auto node = parent_.lock();
    wait_end_ = node->now() + rclcpp::Duration(command->duration);
    return ResultStatus::SUCCEEDED;
}

ResultStatus Wait::onCycleUpdate()
{
    auto node = parent_.lock();
    auto time_left = wait_end_ - node->now();

    if (time_left.nanoseconds() > 0)
    {
        return ResultStatus::RUNNING;
    }
    else
    {
        return ResultStatus::SUCCEEDED;
    }
}
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(behaviors::Wait, gnc_core::Behavior)