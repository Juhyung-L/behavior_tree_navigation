#ifndef BACK_UP_HPP_
#define BACK_UP_HPP_

#include"geometry_msgs/msg/vector3.hpp"

#include "behaviors/timed_behavior.hpp"
#include "gnc_msgs/action/back_up.hpp"

namespace behaviors
{
class BackUp : public TimedBehavior<gnc_msgs::action::BackUp>
{
    using Action = gnc_msgs::action::BackUp;
public:
    BackUp();
    ~BackUp() = default;
    ResultStatus onRun(const std::shared_ptr<const Action::Goal> command) override;
    ResultStatus onCycleUpdate() override;
    void onConfigure() override;
private:
    geometry_msgs::msg::Vector3 getDirectionToNearestObstacle(const geometry_msgs::msg::PoseStamped& current_pose);
    bool isObstacleFree(const geometry_msgs::msg::Twist& cmd_vel, const geometry_msgs::msg::PoseStamped& current_pose);
    geometry_msgs::msg::Vector3 back_up_direction_;
    geometry_msgs::msg::PoseStamped initial_pose_;
    double simulate_ahead_time_;
    rclcpp::Time end_time_;
};
}
#endif