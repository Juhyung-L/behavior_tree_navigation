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
    end_time_ = clock_->now() + rclcpp::Duration(command->duration);
    return ResultStatus::SUCCEEDED;
}

ResultStatus Wait::onCycleUpdate()
{
    auto time_left = end_time_ - clock_->now();

    if (time_left.seconds() > 0)
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