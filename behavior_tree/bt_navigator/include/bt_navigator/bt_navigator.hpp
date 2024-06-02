#ifndef BT_NAVIGATOR_HPP_
#define BT_NAVIGATOR_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"

#include "nav2_util/odometry_utils.hpp"
#include "bt_interface/bt_action_server.hpp"
#include "gnc_core/bt_navigator_base.hpp"

namespace bt_navigator
{
template<class ActionT>
class BTNavigator : public gnc_core::BTNavigatorBase
{
public:
    using Ptr = std::shared_ptr<bt_navigator::BTNavigator<ActionT>>;

    BTNavigator()
    {}

    virtual ~BTNavigator() = default;

    bool on_configure(rclcpp_lifecycle::LifecycleNode::WeakPtr parent,
        std::shared_ptr<nav2_util::OdomSmoother> odom_smoother,
        const std::vector<std::string> & plugin_lib_names,
        std::shared_ptr<tf2_ros::Buffer> tf,
        std::string robot_frame,
        std::string global_frame,
        double transform_tolerance) final
    {
        auto node = parent.lock();
        logger_ = node->get_logger();
        clock_ = node->get_clock();
        odom_smoother_ = odom_smoother;
        tf_ = tf;
        robot_frame_ = robot_frame;
        global_frame_ = global_frame;
        transform_tolerance_ = transform_tolerance;

        std::string bt_xml_filename = getDefaultBTFilepath(parent);

        bt_action_server_ = std::make_unique<bt_interface::BTActionServer<ActionT>>(
            node,
            getName(),
            plugin_lib_names,
            bt_xml_filename,
            std::bind(&bt_navigator::BTNavigator<ActionT>::onGoalReceived, this, std::placeholders::_1),
            std::bind(&bt_navigator::BTNavigator<ActionT>::onLoop, this),
            std::bind(&bt_navigator::BTNavigator<ActionT>::onPreempt, this, std::placeholders::_1),
            std::bind(&bt_navigator::BTNavigator<ActionT>::onCompletion, this, std::placeholders::_1, std::placeholders::_2));
        
        BT::Blackboard::Ptr blackboard = bt_action_server_->getBlackboard();
        blackboard->set<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer", tf_);
        blackboard->set<bool>("initial_pose_received", false);
        blackboard->set<int>("number_recoveries", 0);
        blackboard->set<std::shared_ptr<nav2_util::OdomSmoother>>("odom_smoother", odom_smoother);

        return configure(parent) && bt_action_server_->on_configure();
    }

    bool on_activate() final
    {
        return activate() && bt_action_server_->on_activate();
    }

    bool on_deactivate() final
    {
        return deactivate() && bt_action_server_->on_deactivate();
    }
    
    bool on_cleanup() final
    {
        return cleanup() && bt_action_server_->on_cleanup();
    }

    // these are called in their corresponding on_<action_name>() functions,
    // which are called by whichever class owns this class
    virtual bool configure(rclcpp_lifecycle::LifecycleNode::WeakPtr /*parent*/) {return true;};
    virtual bool activate() {return true;};
    virtual bool deactivate() {return true;};
    virtual bool cleanup() {return true;};

    virtual std::string getDefaultBTFilepath(rclcpp_lifecycle::LifecycleNode::WeakPtr parent) = 0;
    virtual std::string getName() = 0;

    virtual bool onGoalReceived(typename ActionT::Goal::ConstSharedPtr goal) = 0;
    virtual void onLoop() = 0;
    virtual void onPreempt(typename ActionT::Goal::ConstSharedPtr goal) = 0;
    virtual void onCompletion(typename ActionT::Result::SharedPtr result,
        const bt_interface::BTStatus final_bt_status) = 0;
protected:
    std::unique_ptr<bt_interface::BTActionServer<ActionT>> bt_action_server_;
    rclcpp::Logger logger_ = rclcpp::get_logger("bt_navigator");
    rclcpp::Clock::SharedPtr clock_;
    std::shared_ptr<nav2_util::OdomSmoother> odom_smoother_;
    std::shared_ptr<tf2_ros::Buffer> tf_;
    std::string robot_frame_;
    std::string global_frame_;
    double transform_tolerance_;
};
}

#endif