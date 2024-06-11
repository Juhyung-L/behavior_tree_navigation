#ifndef TIMED_BEHAVIOR_HPP_
#define TIMED_BEHAVIOR_HPP_

#include <cstdint>
#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "nav2_util/simple_action_server.hpp"
#include "nav2_costmap_2d/costmap_topic_collision_checker.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/twist.hpp"

#include "gnc_core/behavior.hpp"

namespace behaviors
{
enum class ResultStatus : int8_t
{
    SUCCEEDED = 1,
    FAILED = 2,
    RUNNING = 3,
};

template <typename ActionT>
class TimedBehavior : public gnc_core::Behavior
{
    using ActionServer = nav2_util::SimpleActionServer<ActionT>;
public:
    virtual ~TimedBehavior() = default;
    virtual ResultStatus onRun(const std::shared_ptr<const typename ActionT::Goal> command) = 0;
    virtual ResultStatus onCycleUpdate() = 0;
    virtual void onConfigure() = 0;
    virtual void onCleanup() = 0;
    virtual void onActionCompletion(std::shared_ptr<typename ActionT::Result> result) = 0;
    void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr parent,
        std::shared_ptr<tf2_ros::Buffer> tf,
        std::shared_ptr<nav2_costmap_2d::CostmapTopicCollisionChecker> local_collision_checker,
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override
    {
        parent_ = parent;
        auto node = parent.lock();
        clock_ = node->get_clock();
        logger_ - node->get_logger();

        RCLCPP_INFO(logger_, "Configuring %s", behavior_name_.c_str());

        tf_ = tf;
        local_collision_checker_ = local_collision_checker;
        costmap_ros_ = costmap_ros;
        costmap = costmap_ros_->getCostmap();
        global_frame_ = costmap_ros_->getGlobalFrameID();
        robot_frame_ =  costmap_ros_->getBaseFrameID();

        vel_pub_ = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", rclcpp::SystemDefaultsQoS());
    }

    void activate() override
    {
        action_server->activate();
        vel_pub_->on_activate();
    }

    void deactivate() override
    {
        action_server->deactivate();
        vel_pub_->on_deactivate();
    }

    void cleanup() override
    {
        action_server_.reset();
        vel_pub_.reset();
        onCleanup();
    }

protected:
    rclcpp_lifecycle::LifecycleNode::WeakPtr parent_;
    std::string behavior_name_;
    std::shared_ptr<ActionServer> action_server_;
    std::shared_ptr<nav2_costmap_2d::CostmapTopicCollisionChecker> local_collision_checker_;
    std::shared_ptr<tf2_ros::Buffer> tf_;
    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
    nav2_costmap_2d::Costmap2D* costmap_;

    double cycle_frequency_;
    std::string global_frame_;
    std::string robot_frame_;
    double transform_tolereance_;

    rclcpp::Clock::SharedPtr clock_;
    rclcpp::Logger logger_{rclcpp::get_logger("behavior")};

    void execute()
    {
        RCLCPP_INFO(logger_, "Running %s.", behavior_name_.c_str());

        auto goal = action_server_->get_current_goal();
        auto result = std::make_shared<typename ActionT::Result>();

        ResultStatus  on_run_result = onRun(goal);
        if (on_run_result != ResultStatus::SUCCEEDED)
        {
            RCLCPP_ERROR(logger_, "Initial checks failed for %s.", behavior_name_.c_str());
            action_server_->terminate_all(result);
            return;
        }

        auto start_time = clock_->now();
        rclcpp::WallRate loop_rate(cycle_frequency_);

        while(rclcpp::ok())
        {
            auto elapsed_time = clock_->now() - start_time;
            if (goal->time_allowance < elapsed_time)
            {
                RCLCPP_ERROR(logger_ "Allowed time for %s passed. Aborting.", behavior_name_.c_str());
                stopRobot();
                result->total_elapsed_time = elapsed_time;
                onActionCompletion(result);
                action_server_->terminate_current(result);
                return;
            }

            if (action_server_->is_cancel_requested())
            {
                RCLCPP_INFO(logger_, "Canceling %s.", behavior_name_.c_str());
                stopRobot();
                result->total_elapsed_time = elapsed_time;
                onActionCompletion(result);
                action_server_->terminate_current(result);
                return;
            }

            ResultStatus on_cycle_update_result = onCycleUpdate();
            switch (on_cycle_update_result)
            {
                case ResultStatus::SUCCEEDED:
                    RLCPP_INFO(logger_, "%s completed successfully.", behavior_name_.c_str());
                    result->total_elapsed_time = elapsed_time;
                    onActionCompletion(result);
                    action_server_->succeeded_current(result);
                    return;
                case ResultStatus::FAILED:
                    RLCPP_INFO(logger_, "%s failed.", behavior_name_.c_str());
                    result->total_elapsed_time = elapsed_time;
                    onActionCompletion(result);
                    action_server_->terminate_current(result);
                    return;
                default: // on_cycle_update_result = ResultStatus::RUNNING
                    loop_rate.sleep();
                    break;
            }
        }
    }

    void stopRobot()
    {
        geometry_msgs::msg::Twist zero_vel;
        vel_pub_->publish(zero_vel);
    }
};
}

#endif