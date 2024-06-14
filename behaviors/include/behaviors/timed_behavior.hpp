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
#include "nav2_costmap_2d/costmap_subscriber.hpp"
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
    virtual ResultStatus onRun(const std::shared_ptr<const typename ActionT::Goal> /*command*/) {return ResultStatus::SUCCEEDED;}
    virtual ResultStatus onCycleUpdate() = 0; // only function that is required to be implemented in base class
    virtual void onConfigure() {}
    virtual void onCleanup() {}
    virtual void onActionCompletion(std::shared_ptr<typename ActionT::Result> /*result*/) {}
    void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr parent,
        std::shared_ptr<tf2_ros::Buffer> tf,
        std::shared_ptr<nav2_costmap_2d::CostmapTopicCollisionChecker> collision_checker,
        std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub) override
    {
        RCLCPP_INFO(logger_, "Configuring %s", behavior_name_.c_str());
        
        parent_ = parent;
        auto node = parent.lock();
        clock_ = node->get_clock();
        logger_ = node->get_logger();

        costmap_frame_ = node->get_parameter("costmap_frame").as_string();
        robot_frame_ = node->get_parameter("robot_frame").as_string();
        transform_tolerance_ = node->get_parameter("transform_tolerance").as_double();
        cycle_frequency_ = node->get_parameter("cycle_frequency").as_double();
        
        tf_ = tf;
        collision_checker_ = collision_checker;
        costmap_sub_ = costmap_sub;

        vel_pub_ = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", rclcpp::SystemDefaultsQoS());

        action_server_ = std::make_shared<ActionServer>(
            node,
            action_name_, 
            std::bind(&TimedBehavior::execute, this),
            nullptr,
            std::chrono::milliseconds(500),
            false);
    }

    void activate() override
    {
        action_server_->activate();
        vel_pub_->on_activate();
    }

    void deactivate() override
    {
        action_server_->deactivate();
        vel_pub_->on_deactivate();
    }

    void cleanup() override
    {
        action_server_.reset();
        vel_pub_.reset();
        clock_.reset();
        onCleanup();
    }

protected:
    rclcpp_lifecycle::LifecycleNode::WeakPtr parent_;
    std::string behavior_name_;
    std::string  action_name_;
    std::shared_ptr<ActionServer> action_server_;
    std::shared_ptr<tf2_ros::Buffer> tf_;
    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
    
    std::shared_ptr<nav2_costmap_2d::CostmapTopicCollisionChecker> collision_checker_;
    std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub_;
    std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap_;

    std::string costmap_frame_;
    std::string robot_frame_;
    double transform_tolerance_;
    double cycle_frequency_;

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

        rclcpp::Time start_time = clock_->now();
        rclcpp::WallRate loop_rate(cycle_frequency_);

        while(rclcpp::ok())
        {
            auto elapsed_time = clock_->now() - start_time;

            if (action_server_->is_cancel_requested())
            {
                RCLCPP_INFO(logger_, "Canceling %s.", behavior_name_.c_str());
                stopRobot();
                result->total_elapsed_time = elapsed_time;
                onActionCompletion(result);
                action_server_->terminate_all(result);
                return;
            }

            ResultStatus on_cycle_update_result = onCycleUpdate();
            switch (on_cycle_update_result)
            {
                case ResultStatus::SUCCEEDED:
                    RCLCPP_INFO(logger_, "%s completed successfully.", behavior_name_.c_str());
                    stopRobot();
                    result->total_elapsed_time = elapsed_time;
                    onActionCompletion(result);
                    action_server_->succeeded_current(result);
                    return;
                case ResultStatus::FAILED:
                    RCLCPP_INFO(logger_, "%s failed.", behavior_name_.c_str());
                    stopRobot();
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