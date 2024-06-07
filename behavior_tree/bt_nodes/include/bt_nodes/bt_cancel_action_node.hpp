#ifndef BT_CANCEL_ACTION_NODE_HPP_
#define BT_CANCEL_ACTION_NODE_HPP_

#include <memory>
#include <string>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/action_node.h"

namespace bt_nodes
{
template<class ActionT>
class BTCancelActionNode : public BT::ActionNodeBase
{
    BTCancelActionNode(
        const std::string& xml_tag_name,
        const std::string& action_name,
        const BT::NodeConfiguration& conf)
    : BT::ActionNodeBase(xml_tag_name, conf), action_name_(action_name)
    {
        BT::Blackboard::Ptr blackboard = config().blackboard;
        node_ = blackboard->get<rclcpp::Node::SharedPtr>("node");
        callback_group_ = node_->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive,
            false);
        callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

        server_timeout_ = blackboard->template get<std::chrono::milliseconds>("server_timeout");
        getInput<std::chrono::milliseconds>("server_timeout", server_timeout_);
        wait_for_service_timeout_ = blackboard->template get<std::chrono::milliseconds>("wait_for_service_timeout");

        std::string remapped_action_name;
        if (getInput("server_name", remapped_action_name))
        {
            action_name_ = remapped_action_name;
        }
        createActionClient(action_name_);

        RCLCPP_DEBUG(
            node_->get_logger(), "\"%s\" BtCancelActionNode initialized",
            xml_tag_name.c_str());
    }

    BTCancelActionNode() = delete;

    virtual ~BTCancelActionNode()
    {}

    void createActionClient(const std::string& action_name)
    {
        action_client_ = rclcpp_action::create_client<ActionT>(node_, action_name, callback_group_);

        RCLCPP_DEBUG(node_->get_logger(), "Waiting for \"%s\" action server", action_name.c_str());
        if (!action_client_->wait_for_action_server(wait_for_service_timeout_)) 
        {
            RCLCPP_ERROR(
                node_->get_logger(), "\"%s\" action server not available after waiting for 1 s",
                action_name.c_str());
            throw std::runtime_error(
                    std::string("Action server ") + action_name +
                    std::string(" not available"));
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

    void halt() override
    {
    }

    static BT::PortsList providedPorts()
    {
        return providedBasicPorts({});
    }
    
    BT::NodeStatus tick() override
    {
        // setting the status to RUNNING to notify the BT Loggers (if any)
        setStatus(BT::NodeStatus::RUNNING);

        // cancel all the goals specified before 10ms from current time
        // to avoid async communication error
        rclcpp::Time goal_expiry_time = node_->now() - std::chrono::milliseconds(10);

        auto future_cancel = action_client_->async_cancel_goals_before(goal_expiry_time);

        if (callback_group_executor_.spin_until_future_complete(future_cancel, server_timeout_) !=
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(
                node_->get_logger(),
                "Failed to cancel the action server for %s", action_name_.c_str());
            return BT::NodeStatus::FAILURE;
        }
        return BT::NodeStatus::SUCCESS;
    }

    protected:
    std::string action_name_;
    typename std::shared_ptr<rclcpp_action::Client<ActionT>> action_client_;
    rclcpp::Node::SharedPtr node_;
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp::executors::SingleThreadedExecutor callback_group_executor_;

    // The timeout value while waiting for response from a server when a new action goal is canceled
    std::chrono::milliseconds server_timeout_;
    // The timeout value for waiting for a service to response
    std::chrono::milliseconds wait_for_service_timeout_;
};
}

#endif