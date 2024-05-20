#ifndef BT_ACTION_NODE_HPP_
#define BT_ACTION_NODE_HPP_

#include <memory>
#include <string>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "behaviortree_cpp/action_node.h"

namespace bt_interface
{
template<class ActionT>
class BTActionNode : public BT::ActionNodeBase
{
public:
    BTActionNode(
        const std::string & xml_tag_name,
        const std::string & action_name,
        const BT::NodeConfiguration & conf)
    : BT::ActionNodeBase(xml_tag_name, conf), action_name_(action_name), should_send_goal_(true)
    {
        BT::Blackboard::Ptr blackboard = config().blackboard;

        node_ = blackboard->get<rclcpp::Node::SharedPtr>("node");
        callback_group_ = node_->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive,
            false);
        // each action node gets its own executor, which means they each get their own thread
        callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());
        
        auto bt_loop_duration = blackboard->get<std::chrono::milliseconds>("bt_loop_duration");
        server_timeout_ = blackboard->get<std::chrono::milliseconds>("server_timeout");
        getInput<std::chrono::milliseconds>("server_timeout", server_timeout_);
        wait_for_service_timeout_ = blackboard->get<std::chrono::milliseconds>("wait_for_service_timeout");

        max_timeout_ = std::chrono::duration_cast<std::chrono::milliseconds>(bt_loop_duration * 0.5);

        goal_ = typename ActionT::Goal();
        result_ = typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult();

        // derived class can set "server_name" and remap the action server name
        std::string remapped_action_name;
        if (getInput("server_name", remapped_action_name)) {
        action_name_ = remapped_action_name;
        }
        createActionClient(action_name_);

        RCLCPP_DEBUG(node_->get_logger(), "\"%s\" BtActionNode initialized", xml_tag_name.c_str());
    }

    BtActionNode() = delete;

    virtual ~BtActionNode()
    {}

    void createActionClient(const std::string & action_name)
    {
        action_client_ = rclcpp_action::create_client<ActionT>(node_, action_name, callback_group_);

        RCLCPP_DEBUG(node_->get_logger(), "Waiting for \"%s\" action server", action_name.c_str());
        // wait for wait_for_service_timeout_ seconds before throwing an error
        if (!action_client_->wait_for_action_server(wait_for_service_timeout_)) 
        {
            RCLCPP_ERROR(
                node_->get_logger(), "\"%s\" action server not available after waiting for %f s",
                action_name.c_str(),
                wait_for_service_timeout_.count() / 1000.0);
            throw std::runtime_error(
                std::string("Action server ") + action_name + std::string(" not available"));
        }
    }

    static BT::PortsList providedBasicPorts(BT::PortsList addition)
    {
        BT::PortsList basic = {
            BT::InputPort<std::string>("server_name", "Action server name"),
            BT::InputPort<std::chrono::milliseconds>("server_timeout")};
        basic.insert(addition.begin(), addition.end());

        return basic;
    }

    static BT::PortsList providedPorts()
    {
        return providedBasicPorts({});
    }
    
    // derived class will implement this function
    // this function gets called in the tick function
    virtual void on_tick()
    {}

    virtual void on_wait_for_result(std::shared_ptr<const typename ActionT::Feedback>/*feedback*/)
    {}

    virtual BT::NodeStatus on_success()
    {
        return BT::NodeStatus::SUCCESS;
    }

    virtual BT::NodeStatus on_aborted()
    {
        return BT::NodeStatus::FAILURE;
    }

    virtual BT::NodeStatus on_cancelled()
    {
        return BT::NodeStatus::SUCCESS;
    }

    BT::NodeStatus tick() override
    {
        // isStatusActive returns true if status is not NodeStatus::IDLE and NodeStatus::SKIPPED
       if (!BT::isStatusActive(status()))
       {
            setStatus(BT::NodeStatus::RUNNING);

            should_send_goal_ = true;

            on_tick();

            if (!should_send_goal_)
            {
                return BT::NodeStatus::FAILURE;
            }
            send_new_goal();
       }

       try
       {
            if (future_goal_handle_) // future_goal_handle_ is not null means a goal was sent
            {
                // check time elapsed from when goal was sent
                auto elapsed =
                    (node_->now() - time_goal_sent_).to_chrono<std::chrono::milliseconds>();
                
                if (!is_future_goal_handle_complete(elapsed)) 
                {
                    // return RUNNING if there is still some time before timeout happens
                    if (elapsed < server_timeout_) 
                    {
                        return BT::NodeStatus::RUNNING;
                    }
                    // if server has taken more time than the specified timeout value return FAILURE
                    RCLCPP_WARN(
                        node_->get_logger(),
                        "Timed out while waiting for action server to acknowledge goal request for %s",
                        action_name_.c_str());
                    future_goal_handle_.reset();
                    return BT::NodeStatus::FAILURE;
                }
            }

            if (rclcpp::ok() && !goal_result_available_) 
            {
                // user defined callback. May modify the value of goal_updated_
                on_wait_for_result(feedback_);

                // reset feedback to avoid stale information
                feedback_.reset();

                auto goal_status = goal_handle_->get_status();
                if (goal_updated_ &&
                    (goal_status == action_msgs::msg::GoalStatus::STATUS_EXECUTING ||
                    goal_status == action_msgs::msg::GoalStatus::STATUS_ACCEPTED))
                {
                    goal_updated_ = false;
                    send_new_goal();
                    auto elapsed =
                        (node_->now() - time_goal_sent_).to_chrono<std::chrono::milliseconds>();
                    if (!is_future_goal_handle_complete(elapsed)) 
                    {
                        if (elapsed < server_timeout_) 
                        {
                            return BT::NodeStatus::RUNNING;
                        }
                        RCLCPP_WARN(
                            node_->get_logger(),
                            "Timed out while waiting for action server to acknowledge goal request for %s",
                            action_name_.c_str());
                        future_goal_handle_.reset();
                        return BT::NodeStatus::FAILURE;
                    }
                }

                callback_group_executor_.spin_some();

                // check if, after invoking spin_some(), we finally received the result
                if (!goal_result_available_) 
                {
                    // yield this Action, returning RUNNING
                    return BT::NodeStatus::RUNNING;
                }
            }
       }
       catch (const std::runtime_error& e) 
       {
            if (e.what() == std::string("send_goal failed") ||
                e.what() == std::string("Goal was rejected by the action server"))
            {
                // action related failure that should not fail the tree, but the node
                return BT::NodeStatus::FAILURE;
            } 
            else 
            {
                // internal exception to propagate to the tree
                throw e;
            }
        }

        BT::NodeStatus status;
        switch (result_.code)
        {
            case rclcpp_action::ResultCode::SUCCEEDED:
                status = on_success();
                break;
            case rclcpp_action::ResultCode::ABORTED:
                status = on_aborted();
                break;
            case rclcpp_action::ResultCode::CANCELED:
                status = on_cancelled();
                break;
            default:
                throw std::logic_error("BtActionNode::Tick: invalid status value");
        }
        goal_handle_.reset();
        return status;
    }

    void halt() override
    {
        if (should_cancel_goal()) 
        {
            auto future_result = action_client_->async_get_result(goal_handle_);
            auto future_cancel = action_client_->async_cancel_goal(goal_handle_);
            if (callback_group_executor_.spin_until_future_complete(future_cancel, server_timeout_) !=
                rclcpp::FutureReturnCode::SUCCESS)
            {
                RCLCPP_ERROR(
                    node_->get_logger(),
                    "Failed to cancel action server for %s", action_name_.c_str());
            }

            if (callback_group_executor_.spin_until_future_complete(future_result, server_timeout_) !=
                rclcpp::FutureReturnCode::SUCCESS)
            {
                RCLCPP_ERROR(
                    node_->get_logger(),
                    "Failed to get result for %s in node halt", action_name_.c_str());
            }

            on_cancelled();
        }
        // this is probably redundant, since the parent node
        // is supposed to call it, but we keep it, just in case
        resetStatus();
    }

protected:
    // return true if current goal should be canceled
    bool should_cancel_goal()
    {
        // Shut the node down if it is currently running
        if (status() != BT::NodeStatus::RUNNING) 
        {
            return false;
        }

        // No need to cancel the goal if goal handle is invalid
        if (!goal_handle_) 
        {
            return false;
        }

        callback_group_executor_.spin_some();
        auto status = goal_handle_->get_status();

        // Check if the goal is still executing
        return status == action_msgs::msg::GoalStatus::STATUS_ACCEPTED ||
            status == action_msgs::msg::GoalStatus::STATUS_EXECUTING;
    }

    // send new goal to action server
    void send_new_goal()
    {
        goal_result_available_ = false;
        auto send_goal_options = typename rclcpp_action::Client<ActionT>::SendGoalOptions();
        send_goal_options.result_callback =
            [this](const typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult& result) 
            {
                if (future_goal_handle_) 
                {
                    RCLCPP_DEBUG(
                        node_->get_logger(),
                        "Goal result for %s available, but it hasn't received the goal response yet. "
                        "It's probably a goal result for the last goal request", action_name_.c_str());
                    return;
                }

                // if goal ids do not match, that means the result is for an older goal
                if (goal_handle_->get_goal_id() == result.goal_id) 
                {
                    goal_result_available_ = true;
                    result_ = result;
                    emitWakeUpSignal(); // tick tree again (why?)
                }
            };
        send_goal_options.feedback_callback =
            [this](typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr,
                const std::shared_ptr<const typename ActionT::Feedback> feedback) 
            {
                feedback_ = feedback;
                emitWakeUpSignal(); // tick tree again (why?)
            };
        future_goal_handle_ = 
            std::make_shared<std::shared_future<typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr>>(
                action_client_->async_send_goal(goal_, send_goal_options));
        time_goal_sent_ = node_->now();
    }

    bool is_future_goal_handle_complete(std::chrono::milliseconds& elapsed)
    {
        auto remaining = server_timeout_ - elapsed;

        // server has already timed out, no need to sleep
        if (remaining <= std::chrono::milliseconds(0)) 
        {
            future_goal_handle_.reset();
            return false;
        }

        // max_timeout_ = 0.5 * bt_loop_duration
        auto timeout = remaining > max_timeout_ ? max_timeout_ : remaining;
        auto result =
            callback_group_executor_.spin_until_future_complete(*future_goal_handle_, timeout);
        elapsed += timeout;

        if (result == rclcpp::FutureReturnCode::INTERRUPTED) 
        {
            future_goal_handle_.reset();
            throw std::runtime_error("send_goal failed");
        }

        if (result == rclcpp::FutureReturnCode::SUCCESS) 
        {
            goal_handle_ = future_goal_handle_->get();
            future_goal_handle_.reset();
            if (!goal_handle_) 
            {
                throw std::runtime_error("Goal was rejected by the action server");
            }
            return true;
        }
        return false;
    }

    void increment_recovery_count()
    {
        int recovery_count = 0;
        [[maybe_unused]] auto res = config().blackboard->get("number_recoveries", recovery_count);
        recovery_count += 1;
        config().blackboard->set("number_recoveries", recovery_count);
    }

    std::string action_name_;
    typename std::shared_ptr<rclcpp_action::Client<ActionT>> action_client_;

    typename ActionT::Goal goal_;
    bool goal_updated_{false};
    bool goal_result_available_{false};
    typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr goal_handle_;
    typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult result_;
    std::shared_ptr<const typename ActionT::Feedback> feedback_;

    rclcpp::Node::SharedPtr node_;
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp::executors::SingleThreadedExecutor callback_group_executor_;

    // timeout for waiting for a response from the action server when a new goal is sent or canceled
    std::chrono::milliseconds server_timeout_;
    // timeout for BT loop execution
    std::chrono::milliseconds max_timeout_;
    // timeout for waiting for a service to respond
    std::chrono::milliseconds wait_for_service_timeout_;

    std::shared_ptr<std::shared_future<typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr>> future_goal_handle_;
    rclcpp::Time time_goal_sent_;
    bool should_send_goal_;
};
}

#endif