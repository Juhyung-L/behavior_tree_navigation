#ifndef BT_ACTION_SERVER_HPP_
#define BT_ACTION_SERVER_HPP_

#include <memory>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include "nav2_util/simple_action_server.hpp"

#include "bt_navigators/bt_engine.hpp"

namespace bt_navigators
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
        const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent,
        const std::string& action_name,
        const std::vector<std::string>& plugin_lib_names,
        const std::string& default_bt_xml_filename,
        OnGoalReceivedCallback on_goal_received_callback,
        OnLoopCallback on_loop_callback,
        OnPreemptCallback on_preempt_callback,
        OnCompletionCallback on_completion_callback)
    : action_name_(action_name)
    , default_bt_xml_filename_(default_bt_xml_filename)
    , plugin_lib_names_(plugin_lib_names)
    , parent_(parent)
    , on_goal_received_callback_(on_goal_received_callback)
    , on_loop_callback_(on_loop_callback)
    , on_preempt_callback_(on_preempt_callback)
    , on_completion_callback_(on_completion_callback)
    {
        auto node = parent_.lock();
        logger_ = node->get_logger();
        clock_ = node->get_clock();

        node->declare_parameter("bt_loop_duration", 10);
        node->declare_parameter("server_timeout", 20);
        node->declare_parameter("wait_for_service_timeout", 1000);
        node->declare_parameter("action_server_result_timeout", 900.0);
    }

    ~BTActionServer() {}

    bool on_configure()
    {
        auto node = parent_.lock();

        client_node_ = std::make_shared<rclcpp::Node>("client_node", rclcpp::NodeOptions());

        double action_server_result_timeout =
        node->get_parameter("action_server_result_timeout").as_double();
        rcl_action_server_options_t server_options = rcl_action_server_get_default_options();
        server_options.result_timeout.nanoseconds = RCL_S_TO_NS(action_server_result_timeout);

        action_server_ = std::make_shared<ActionServer>(
            node->get_node_base_interface(),
            node->get_node_clock_interface(),
            node->get_node_logging_interface(),
            node->get_node_waitables_interface(),
            action_name_, 
            std::bind(&BTActionServer<ActionT>::executeCallback, this),
            nullptr,
            std::chrono::milliseconds(500),
            false,
            server_options);

        int time;
        node->get_parameter("bt_loop_duration", time);
        bt_loop_duration_ = std::chrono::milliseconds(time);
        node->get_parameter("server_timeout", time);
        server_timeout_ = std::chrono::milliseconds(time);
        node->get_parameter("wait_for_service_timeout", time);
        wait_for_service_timeout_ = std::chrono::milliseconds(time);
        
        bt_ = std::make_unique<bt_navigators::BTEngine>(plugin_lib_names_);

        // make blackboard and store necessary shared variables
        blackboard_ = BT::Blackboard::create();
        blackboard_->set<rclcpp::Node::SharedPtr>("node", client_node_);
        blackboard_->set<std::chrono::milliseconds>("server_timeout", server_timeout_);
        blackboard_->set<std::chrono::milliseconds>("bt_loop_duration", bt_loop_duration_);
        blackboard_->set<std::chrono::milliseconds>("wait_for_service_timeout", wait_for_service_timeout_);
        return true;
    }

    bool on_activate()
    {
        if (!loadBehaviorTree(default_bt_xml_filename_))
        {
            return false;
        }
        groot_pub_ = std::make_unique<BT::Groot2Publisher>(tree_);
        action_server_->activate();
        return true;
    }

    bool on_deactivate()
    {
        groot_pub_.reset();
        action_server_->deactivate();
        return true;
    }

    bool on_cleanup()
    {
        client_node_.reset();
        action_server_.reset();
        plugin_lib_names_.clear();
        current_bt_xml_filename_.clear();
        default_bt_xml_filename_.clear();
        blackboard_.reset();
        bt_->haltAllActions(tree_);
        bt_.reset();
        return true;
    }

    bool loadBehaviorTree(const std::string& input_bt_xml_filename = "")
    {
        std::string bt_xml_filename;
        if (input_bt_xml_filename.empty())
        {
            bt_xml_filename = default_bt_xml_filename_;
        }
        else
        {
            bt_xml_filename = input_bt_xml_filename;
        }

        // use previous BT if it is the existing one
        if (current_bt_xml_filename_ == bt_xml_filename) {
            RCLCPP_DEBUG(logger_, "BT will not be reloaded as the given xml is already loaded");
            return true;
        }

        // read the input BT XML from the specified file into a string
        std::ifstream xml_file(bt_xml_filename);
        if (!xml_file.good()) {
            RCLCPP_ERROR(logger_, "Couldn't open input XML file: %s", bt_xml_filename.c_str());
            return false;
        }
        
        try
        {
            // this only sets the blackboard of the root node
            tree_ = bt_->createTreeFromFile(bt_xml_filename, blackboard_);
            // share variables to all subtree blackboards
            for (auto& subtree : tree_.subtrees) 
            {
                subtree->blackboard->set<rclcpp::Node::SharedPtr>("node", client_node_);
                subtree->blackboard->set<std::chrono::milliseconds>("server_timeout", server_timeout_);
                subtree->blackboard->set<std::chrono::milliseconds>("bt_loop_duration", bt_loop_duration_);
                subtree->blackboard->set<std::chrono::milliseconds>("wait_for_service_timeout", wait_for_service_timeout_);
            }
        }
        catch (std::exception& ex)
        {
            RCLCPP_ERROR(logger_, "Exception while loading BT: %s", ex.what());
            return false;
        }

        current_bt_xml_filename_ = bt_xml_filename;
        return true;
    }

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

    const BT::Tree& getTree() const
    {
        return tree_;
    }

    void haltTree()
    {
        tree_.haltTree();
    }

private:
    void executeCallback()
    {
        if (!on_goal_received_callback_(action_server_->get_current_goal()))
        {
            action_server_->terminate_current();
            return;
        }

        auto on_loop = [&]() 
        {
            if (action_server_->is_preempt_requested() && on_preempt_callback_)
            {
                on_preempt_callback_(action_server_->get_pending_goal());
            }
            on_loop_callback_();
        };

        auto is_canceling = [&]()
        {
            if (action_server_ == nullptr)
            {
                RCLCPP_DEBUG(logger_, "Action server unavailable. Canceling.");
                return true;
            }
            if (!action_server_->is_server_active())
            {
                RCLCPP_DEBUG(logger_, "Action server is inactive. Canceling.");
                return true;
            }
            return action_server_->is_cancel_requested();
        };

        // blocking function to run until the tree finishes executing or something happens with ROS
        BTStatus tree_result = bt_->run(&tree_, on_loop, is_canceling, bt_loop_duration_);
        
        // make sure tree stopped running
        // if all nodes were implemented correctly, this is not needed
        bt_->haltAllActions(tree_);

        auto result = std::make_shared<typename ActionT::Result>();
        on_completion_callback_(result, tree_result);
        switch(tree_result)
        {
            case BTStatus::SUCCEEDED:
                RCLCPP_INFO(logger_, "Goal succeeded");
                action_server_->succeeded_current(result);
                break;
            case BTStatus::FAILED:
                RCLCPP_ERROR(logger_, "Goal failed");
                action_server_->terminate_current(result);
                break;
            case BTStatus::CANCELED:
                RCLCPP_INFO(logger_, "Goal canceled");
                action_server_->terminate_all(result);
                break;
        }
    }

    std::string action_name_;
    std::shared_ptr<ActionServer> action_server_;

    BT::Tree tree_;
    BT::Blackboard::Ptr blackboard_;

    std::string current_bt_xml_filename_;
    std::string default_bt_xml_filename_;

    std::unique_ptr<bt_navigators::BTEngine> bt_;
    std::vector<std::string> plugin_lib_names_;

    rclcpp::Node::SharedPtr client_node_;
    rclcpp_lifecycle::LifecycleNode::WeakPtr parent_;

    rclcpp::Clock::SharedPtr clock_;
    rclcpp::Logger logger_{rclcpp::get_logger("bt_action_server")};

    // all three timeouts are uploaded to behavior tree blackboard
    std::chrono::milliseconds bt_loop_duration_; // period of the while loop inside  bt_->run()
    std::chrono::milliseconds server_timeout_;
    std::chrono::milliseconds wait_for_service_timeout_;
    
    OnGoalReceivedCallback on_goal_received_callback_;
    OnLoopCallback on_loop_callback_;
    OnPreemptCallback on_preempt_callback_;
    OnCompletionCallback on_completion_callback_;

    std::unique_ptr<BT::Groot2Publisher> groot_pub_;
};
}

#endif