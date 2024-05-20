#ifndef BT_ACTION_SERVER_HPP_
#define BT_ACTION_SERVER_HPP_

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "behaviortree_cpp/bt_factory.h"

#include "bt_interface/bt_engine.hpp"
#include "nav2_util/simple_action_server.hpp"

namespace bt_interface
{
template<class ActionT>
class BTActionServer
{
public:
    using ActionServer = nav2_util::SimpleActionServer<ActionT>;

    typedef std::function<bool (typename ActionT::Goal::ConstSharedPtr)> OnGoalReceivedCallback;
    typedef std::function<void ()> OnLoopCallback;
    typedef std::function<void (typename ActionT::Goal::ConstSharedPtr)> OnPreemptCallback;
    typedef std::function<void (typename ActionT::Result::SharedPtr, BTStatus)> OnCompletionCallback;

    explicit BTActionServer(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
        const std::string & action_name,
        const std::vector<std::string> & plugin_lib_names,
        const std::string & default_bt_xml_filename,
        OnGoalReceivedCallback on_goal_received_callback,
        OnLoopCallback on_loop_callback,
        OnPreemptCallback on_preempt_callback,
        OnCompletionCallback on_completion_callback);

    ~BTActionServer();

    bool on_configure();
    bool on_activate();
    bool on_deactivate();
    bool on_cleanup();

    bool loadBehaviorTree(const std::string & bt_xml_filename = "");

    BT::Blackboard::Ptr getBlackboard() const
    {
        return blackboard_;
    }

    std::string getCurrentBTFilename() const
    {
        return current_bt_xml_filename_;
    }

    const std::shared_ptr<const typename ActionT::Goal> acceptPendingGoal()
    {
        return action_server_->accept_pending_goal();
    }

    void terminatePendingGoal()
    {
        action_server_->terminate_pending_goal();
    }

    const std::shared_ptr<const typename ActionT::Goal> getCurrentGoal() const
    {
        return action_server_->get_current_goal();
    }

    const std::shared_ptr<const typename ActionT::Goal> getPendingGoal() const
    {
        return action_server_->get_pending_goal();
    }

    void publishFeedback(typename std::shared_ptr<typename ActionT::Feedback> feedback)
    {
        action_server_->publish_feedback(feedback);
    }

    const BT::Tree & getTree() const
    {
        return tree_;
    }

    void haltTree()
    {
        tree_.haltTree();
    }

private:
    void executeCallback();
    std::string action_name_;

    std::shared_ptr<ActionServer> action_server_;

    BT::Tree tree_;

    BT::Blackboard::Ptr blackboard_;

    std::string current_bt_xml_filename_;
    std::string bt_xml_filename_;

    std::unique_ptr<bt_interface::BTEngine> bt_;
    std::vector<std::string> plugin_lib_names_;

    rclcpp::Node::SharedPtr client_node_;
    rclcpp_lifecycle::LifecycleNode::WeakPtr node_;

    rclcpp::Clock::SharedPtr clock_;
    rclcpp::Logger logger_;

    // all three timeouts are uploaded to behavior tree blackboard
    std::chrono::milliseconds bt_loop_duration_; // period of the while loop inside  bt_->run()
    std::chrono::milliseconds server_timeout_;
    std::chrono::milliseconds wait_for_service_timeout_;
    
    bool always_reload_bt_xml_ = false;

    OnGoalReceivedCallback on_goal_received_callback_;
    OnLoopCallback on_loop_callback_;
    OnPreemptCallback on_preempt_callback_;
    OnCompletionCallback on_completion_callback_;
};
}

#endif