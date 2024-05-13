// Copyright (c) 2019 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef SIMPLE_ACTION_SERVER_HPP_
#define SIMPLE_ACTION_SERVER_HPP_

#include <string>
#include <functional>
#include <chrono>
#include <mutex>
#include <future>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "util/node_thread.hpp"

namespace util
{
using namespace std::chrono_literals;
using namespace std::placeholders;

template<typename ActionT>
class SimpleActionServer
{
public:
    // function to complete this action server's main work
    typedef std::function<void ()> ExecuteCallback;

    // functo to call once ExecuteCallback is completed
    typedef std::function<void ()> CompletionCallback;

    template<typename NodeT>
    explicit SimpleActionServer(
        NodeT node,
        const std::string& action_name,
        ExecuteCallback execute_callback,
        CompletionCallback completion_callback = nullptr,
        std::chrono::milliseconds server_timeout = 500ms
        bool spin_thread = false,
        const rcl_action_server_options_t& options = rcl_action_server_get_default_options())
    : SimpleActionServer(
        node->get_node_base_interface(),
        node->get_node_clock_interface(),
        node->get_node_logging_interface(),
        node->get_node_waitables_interface(),
        action_name, execute_callback, server_timeout, spin_thread, options)
    {}

    explicit SimpleActionServer(
        rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_interface,
        rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock_interface,
        rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_interface,
        rclcpp::node_interfaces::NodeWaitablesInterface::SharedPtr node_waitables_interface,
        const std::string& action_name,
        ExecuteCallback execute_callback,
        CompletionCallback completion_callback = nullptr,
        std::chrono::milliseconds server_timeout = 500ms,
        bool spin_thread = false,
        const rcl_action_server_options_t& options = rcl_action_server_get_default_options())
    : node_base_interface_(node_base_interface)
    , node_clock_interface_(node_clock_interface)
    , node_logging_interface_(node_logging_interface)
    , node_waitables_interface_(node_waitables_interface)
    , action_name_(action_name)
    , execute_callback_(execute_callback)
    , completion_callback_(completion_callback)
    , server_timeout_(server_timeout)
    , spin_thread_(spin_thread)
    {
        if (spin_thread_)
        {
            callback_group_ = node_base_interface->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
        }
        action_server_ = rclcpp_action::create_server<ActionT>(
            node_base_interface_,
            node_clock_interface_,
            node_logging_interface_,
            node_waitables_interface_,
            action_name_,
            std::bind(&SimpleActionServer::handleGoal, this, _1, _2),
            std::bind(&SimpleActionServer::handleCancel, this, _1),
            std::bind(&SimpleActionServer::handleAccepted, this, _1),
            options,
            callback_group_);
        if (spin_thread_) 
        {
            executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
            executor_->add_callback_group(callback_group_, node_base_interface_);
            executor_thread_ = std::make_unique<util::NodeThread>(executor_);
        }
    }

    void activate()
    {
        std::lock_guard<std::recursive_mutex> lock(update_mutex_);
        server_active_ = true;
        stop_execution_ = false;
    }

    void deactivate()
    {
        {
            std::lock_guard<std::recursive_mutex> lock(update_mutex_);
            server_active_ = false;
            stop_execution_ = true;
        }

        if (!isServerRunning())
        {
            return;
        }
        // requested to deactivate when there is an async task running
        warn_msg("Requested to deactivate when there is a goal being executed");

        auto start_time = std::chrono::steady_clock::now();
        while (execution_future_.wait_for(100ms) != std::future_status::ready)
        {
            info_msg("Waiting for goal to finish executing");
            if (std::chrono::steady_clock::now() - start_time >= server_timeout_) // wait for longer than server_timeout_
            {
                terminateAll();
                completion_callback_();
                throw std::runtime_error("Could not stop the action server's executing thread");
            }
        }
    }

    void terminateAll(typename std::shared_ptr<typename ActionT::Result> result =
        std::make_shared<typename ActionT::Result>())
    {
        std::lock_guard<std::recursive_mutex> lock(update_mutex_);
        terminate(current_handle_, result);
        terminate(pending_handle_, result);
        preempt_requested_ = false;
    }

    void terminateCurrent(typename std::shared_ptr<typename ActionT::Result> result =
        std::make_shared<typename ActionT::Result>())
    {
        std::lock_guard<std::recursive_mutex> lock(update_mutex_);
        terminate(current_handle_, result);
    }

    void terminatePendingGoal()
    {
        if (!isHandleActive(pending_handle_))
        {
            error_msg("Attempting to terminate pending goal that does not exist");
            return;
        }

        terminate(pending_handle_);
        preempt_requested_ = false;
    }

    void succeedCurrent(typename std::shared_ptr<typename ActionT::Result> result =
        std::make_shared<typename ActionT::Result>())
    {
        std::lock_guard<std::recursive_mutex> lock(update_mutex_);
        if (isHandleActive)
        {
            current_handle_->succeed(result);
            current_handle_.reset();
        }
    }

    void publishFeedback(typename std::shared_ptr<typename ActionT::Feedback> feedback)
    {
        if (!isHandleActive(current_handle_))
        {
            error_msg("Trying to publish feedback but there is no goal executing");
            return;
        }
        current_handle_->publish_feedback(feedback);
    }

    bool isServerActive()
    {
        std::lock_guard<std::recursive_mutex> lock(update_mutex_);
        return server_active_;
    }

    bool isPreemptRequested()
    {
        std::lock_guard<std::recursive_mutex> lock(update_mutex_);
        return preempt_requested_;
    }

    const std::shared_ptr<const typename ActionT::Goal> getCurrentGoal()
    {
        std::lock_guard<std::recursive_mutex> lock(update_mutex_);
        if (!isHandleActive(current_handle_))
        {
            error_msg("There is no available goal");
            return std::shared_ptr<const typename ActionT::Goal>();
        }
        return current_handle_->get_goal();
    }

    const rclcpp_action::GoalUUID getCurrentGoalID()
    {
        std::lock_guard<std::recursive_mutex> lock(update_mutex_);
        if (!isHandleActive(current_handle_))
        {
            error_msg("There is no available goal");
            return std::shared_ptr<const typename ActionT::Goal>();
        }
        return current_handle_->get_goal_id();
    }

    const std::shared_ptr<const typename ActionT::Goal> getPendingGoal()
    {
        std::lock_guard<std::recursive_mutex> lock(update_mutex_);

        if (!isHandleActive(pending_handle_)) 
        {
            error_msg("There is no available pending goal");
            return std::shared_ptr<const typename ActionT::Goal>();
        }
        return pending_handle_->get_goal();
    }

    bool isCancelRequested()
    {
        std::lock_guard<std::recursive_mutex> lock(update_mutex_);
        if (current_handle_ == nullptr)
        {
            return false;
        }
        return current_handle_->is_canceling();
    }

protected:
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp_action::Server<ActionT>::SharedPtr action_server_;
    std::recursive_mutex update_mutex_;
    bool server_active_{false};
    bool preempt_requested_{false}; // flag to indicate if a goal is in the pending slot
    bool stop_execution_{false};
    bool spin_thread_;
    std::chrono::milliseconds server_timeout_;
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_interface_;
    rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock_interface_;
    rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_interface_;
    rclcpp::node_interfaces::NodeWaitablesInterface::SharedPtr node_waitables_interface_;
    std::string action_name_;
    ExecuteCallback execute_callback_;
    CompletionCallback completion_callback_;
    std::future<void> execution_future_;
    std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> current_handle_;
    std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> pending_handle_;
    rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
    std::unique_ptr<util::NodeThread> executor_thread_;

    rclcpp_action::GoalResponse handleGoal(
        const rclcpp_action::GoalUUID& /*uuid*/,
        std::shared_ptr<const typename ActionT::Goal> /*goal*/)
    {
        if (!server_active_)
        {
            return rclcpp_action::GoalResponse::REJECT;
            debug_msg("Goal rejected");
        }
        debug_msg("Goal received. Executing action");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }
    
    rclcpp_action::CancelResponse handleCancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> handle)
    {
        std::lock_guard<std::recursive_mutex> lock(update_mutex_);
        if (!handle->is_active())
        {
            warn_msg(
                "Tried to cancel goal but handle is inactive");
            return rclcpp_action::CancelResponse::REJECT;
        }
        debug_msg("Canceling goal");
        return rclcpp_action::CancelResponse::ACCEPT;
        // all this function does is make handle->is_canceling() return true
    }

    void handleAccepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> handle)
    {
        std::lock_guard<std::recursive_mutex> lock(update_mutex_);
        if (isHandleActive(current_handle_) || isServerRunning()) // check if action server is already running
        {
            debug_msg("An older goal is active, moving the new goal to a pending slot");
            if (isHandleActive(pending_handle_))
            {
                debug_msg(
                    "The pending slot is occupied",
                    "Terminating previous pending goal and replacing with received goal");
                terminate(pending_handle_);
            }
            pending_handle_ = handle;
            preempt_requested_ = true;
        }
        else
        {
            if (isHandleActive(pending_handle_))
            {
                error_msg("No current goal, but there is a pending goal. This shouldn't happen. Terminating pending goal");
                terminate(pending_handle_);
                preempt_requested_ = false;
            }
        }
        current_handle_ = handle;
        debug_msg("Executing goal");
        execution_future_ = std::async(std::launch::async, [this](){work();});
    }

    void work()
    {
        while (rclcpp::ok() && !stop_execution_ && isHandleActive(current_handle_))
        {
            try
            {
                execute_callback_();
            }
            catch (std::exception& ex)
            {
                error_msg("Executing function threw error: " + ex.what());
                terminateAll();
                completion_callback_();
                return;
            }

            // block processing of new goal handle with mutex
            std::lock_guard<std::recursive_mutex> lock(update_mutex_);

            if (stop_execution_)
            {
                warn_msg("Stopping execution");
                terminate_all();
                completion_callback_();
                break;
            }

            if (isHandleActive(current_handle_))
            {
                warn_msg("Current goal did not complete");
                terminate(current_handle_);
                completion_callback_();
            }

            if (isHandleActive(pending_handle_))
            {
                debug_msg("Executing pending goal");
            }
            else
            {
                debug_msg("Done processing all goals");
                break;
            }
        }
    }

    const std::shared_ptr<const typename ActionT::Goal> acceptPendingGoal()
    {
        std::lock_guard<std::recursive_mutex> lock(update_mutex_);
        if (!isHandleActive(pending_handle_))
        {
            error_msg("Attempting to accept pending goal that does not exist");
            return std::shared_ptr<const typename ActionT::Goal>();
        }

        if (isHandleActive(current_handle_) && pending_handle_ != current_handle_)
        {
            terminate(current_handle_);
        }

        current_handle_ = pending_handle_;
        pending_handle_.reset();
        preempt_requested_ = false;
        
        return current_handle_->get_goal();
    }

    void terminate(std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> & handle,
        typename std::shared_ptr<typename ActionT::Result> result =
            std::make_shared<typename ActionT::Result>())
    {
        std::lock_guard<std::recursive_mutex> lock(update_mutex_);
        if (isHandleActive(handle))
        {
            if (handle->is_canceling())
            {
                if (handle->is_canceling())
                {
                    warn_msg("Canceling handle");
                    handle->canceled(result);
                }
                else
                {
                    warn_msg("Aboring handle");
                    handle->abort(result);
                }
                handle.reset();
            }
        }
    }

    bool isHandleActive(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> handle)
    {
        return handle != nullptr && handle->is_active();
    }

    bool isServerRunning()
    {
        // valid future means a task was launched using std::async
        // wait for the future to complete for 0 seconds, if it times out there is a task running
        return execution_future_.valid() && 
            (execution_future_.wait_for(std::chrono::milliseconds(0)) == std::future_status::timeout);
    }

    void info_msg(const std::string & msg) const
    {
        RCLCPP_INFO(node_logging_interface_->get_logger(),
            "[%s] [ActionServer] %s", action_name_.c_str(), msg.c_str());
    }

    void debug_msg(const std::string & msg) const
    {
        RCLCPP_DEBUG(node_logging_interface_->get_logger(),
            "[%s] [ActionServer] %s", action_name_.c_str(), msg.c_str());
    }

    void warn_msg(const std::string & msg) const
    {
        RCLCPP_WARN(node_logging_interface_->get_logger(),
            "[%s] [ActionServer] %s", action_name_.c_str(), msg.c_str());
    }

    void error_msg(const std::string & msg) const
    {
        RCLCPP_ERROR(node_logging_interface_->get_logger(),
            "[%s] [ActionServer] %s", action_name_.c_str(), msg.c_str());
    }
};
}

#endif