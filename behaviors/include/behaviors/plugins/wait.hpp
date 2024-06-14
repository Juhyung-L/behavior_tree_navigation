#ifndef WAIT_HPP_
#define WAIT_HPP_

#include "rclcpp/rclcpp.hpp"

#include "behaviors/timed_behavior.hpp"
#include "gnc_msgs/action/wait.hpp"
namespace behaviors
{
class Wait : public TimedBehavior<gnc_msgs::action::Wait>
{
    using Action = gnc_msgs::action::Wait;
public:
    Wait();
    ~Wait() = default;
    ResultStatus onRun(const std::shared_ptr<const Action::Goal> command) override;
    ResultStatus onCycleUpdate() override;
private:
    rclcpp::Time end_time_;
};
}
#endif