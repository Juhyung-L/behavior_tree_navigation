#ifndef KINEMATICS_PARAMETERS_HPP_
#define KINEMATICS_PARAMETERS_HPP_

#include <memory>

#include "nav2_util/lifecycle_node.hpp"

namespace dwa_core
{
/**
 * @class KinematicsParameters
 * @brief A class that stores the maximum/minimum velocities & acceleration of the robot
*/

class KinematicsParameters
{
public:
    KinematicsParameters(const nav2_util::LifecycleNode::WeakPtr parent, const std::string& plugin_name)
    : parent_(parent)
    , plugin_name_(plugin_name)
    {
        auto node = parent_.lock();
        node->declare_parameter(plugin_name_ + ".min_vel_x", rclcpp::ParameterValue(0.0));
        node->declare_parameter(plugin_name_ + ".max_vel_x", rclcpp::ParameterValue(0.0));
        node->declare_parameter(plugin_name_ + ".acc_x", rclcpp::ParameterValue(0.0));
        node->declare_parameter(plugin_name_ + ".decel_x", rclcpp::ParameterValue(0.0));

        node->declare_parameter(plugin_name_ + ".min_vel_y", rclcpp::ParameterValue(0.0));
        node->declare_parameter(plugin_name_ + ".max_vel_y", rclcpp::ParameterValue(0.0));
        node->declare_parameter(plugin_name_ + ".acc_y", rclcpp::ParameterValue(0.0));
        node->declare_parameter(plugin_name_ + ".decel_y", rclcpp::ParameterValue(0.0));

        node->declare_parameter(plugin_name_ + ".min_vel_theta", rclcpp::ParameterValue(0.0));
        node->declare_parameter(plugin_name_ + ".max_vel_theta", rclcpp::ParameterValue(0.0));
        node->declare_parameter(plugin_name_ + ".acc_theta", rclcpp::ParameterValue(0.0));
        node->declare_parameter(plugin_name_ + ".decel_theta", rclcpp::ParameterValue(0.0));
    }

    void initialize()
    {
        auto node = parent_.lock();
        min_vel_x_ = node->get_parameter(plugin_name_ + ".min_vel_x").as_double();
        max_vel_x_ = node->get_parameter(plugin_name_ + ".max_vel_x").as_double();
        acc_x_ = node->get_parameter(plugin_name_ + ".acc_x").as_double();
        decel_x_ = node->get_parameter(plugin_name_ + ".decel_x").as_double();

        min_vel_y_ = node->get_parameter(plugin_name_ + ".min_vel_y").as_double();
        max_vel_y_ = node->get_parameter(plugin_name_ + ".max_vel_y").as_double();
        acc_y_ = node->get_parameter(plugin_name_ + ".acc_y").as_double();
        decel_y_ = node->get_parameter(plugin_name_ + ".decel_y").as_double();

        min_vel_theta_ = node->get_parameter(plugin_name_ + ".min_vel_theta").as_double();
        max_vel_theta_ = node->get_parameter(plugin_name_ + ".max_vel_theta").as_double();
        acc_theta_ = node->get_parameter(plugin_name_ + ".acc_theta").as_double();
        decel_theta_ = node->get_parameter(plugin_name_ + ".decel_theta").as_double();
    }

    double getMinVelX() {return min_vel_x_;}
    double getMaxVelX() {return max_vel_x_;}
    double getAccX() {return acc_x_;}
    double getDecelX() {return decel_x_;}

    double getMinVelY() {return min_vel_y_;}
    double getMaxVelY() {return max_vel_y_;}
    double getAccY() {return acc_y_;}
    double getDecelY() {return decel_y_;}

    double getMinVelTheta() {return min_vel_theta_;}
    double getMaxVelTheta() {return max_vel_theta_;}
    double getAccTheta() {return acc_theta_;}
    double getDecelTheta() {return decel_theta_;}

    using SharedPtr = std::shared_ptr<KinematicsParameters>;

private:
    double min_vel_x_, max_vel_x_, acc_x_, decel_x_;
    double min_vel_y_, max_vel_y_, acc_y_, decel_y_;
    double min_vel_theta_, max_vel_theta_, acc_theta_, decel_theta_;

    rclcpp_lifecycle::LifecycleNode::WeakPtr parent_;
    std::string plugin_name_;
};
}

#endif