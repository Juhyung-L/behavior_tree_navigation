#ifndef KINEMATICS_PARAMETERS_HPP_
#define KINEMATICS_PARAMETERS_HPP_

#include <memory>

#include "nav2_util/lifecycle_node.hpp"

using namespace std::placeholders;


namespace dwa_core
{
/**
 * @class KinematicsParameters
 * @brief A class that stores the maximum/minimum velocities & acceleration of the robot
*/

class KinematicsParameters
{
public:
    KinematicsParameters(nav2_util::LifecycleNode::SharedPtr nh)
    {
        nh->declare_parameter("min_vel_x", rclcpp::ParameterValue(0.0));
        nh->declare_parameter("max_vel_x", rclcpp::ParameterValue(0.0));
        nh->declare_parameter("acc_x", rclcpp::ParameterValue(0.0));
        nh->declare_parameter("decel_x", rclcpp::ParameterValue(0.0));

        nh->declare_parameter("min_vel_y", rclcpp::ParameterValue(0.0));
        nh->declare_parameter("max_vel_y", rclcpp::ParameterValue(0.0));
        nh->declare_parameter("acc_y", rclcpp::ParameterValue(0.0));
        nh->declare_parameter("decel_y", rclcpp::ParameterValue(0.0));

        nh->declare_parameter("min_vel_theta", rclcpp::ParameterValue(0.0));
        nh->declare_parameter("max_vel_theta", rclcpp::ParameterValue(0.0));
        nh->declare_parameter("acc_theta", rclcpp::ParameterValue(0.0));
        nh->declare_parameter("decel_theta", rclcpp::ParameterValue(0.0));
        
        min_vel_x_ = nh->get_parameter("min_vel_x").as_double();
        max_vel_x_ = nh->get_parameter("max_vel_x").as_double();
        acc_x_ = nh->get_parameter("acc_x").as_double();
        decel_x_ = nh->get_parameter("decel_x").as_double();

        min_vel_y_ = nh->get_parameter("min_vel_y").as_double();
        max_vel_y_ = nh->get_parameter("max_vel_y").as_double();
        acc_y_ = nh->get_parameter("acc_y").as_double();
        decel_y_ = nh->get_parameter("decel_y").as_double();

        min_vel_theta_ = nh->get_parameter("min_vel_theta").as_double();
        max_vel_theta_ = nh->get_parameter("max_vel_theta").as_double();
        acc_theta_ = nh->get_parameter("acc_theta").as_double();
        decel_theta_ = nh->get_parameter("decel_theta").as_double();

        dyn_params_handler_ = nh->add_on_set_parameters_callback(
            std::bind(&KinematicsParameters::dynamicParametersCallback,  this, _1)
        );
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

    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;

    rcl_interfaces::msg::SetParametersResult
    dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;

        for (auto parameter : parameters) {
            const auto & type = parameter.get_type();
            const auto & name = parameter.get_name();

            if (type == rclcpp::ParameterType::PARAMETER_DOUBLE) {
                if (name == "min_vel_x") {
                    min_vel_x_ = parameter.as_double();
                } else if (name == "max_vel_x") {
                    max_vel_x_ = parameter.as_double();
                } else if (name == "acc_x") {
                    acc_x_ = parameter.as_double();
                } else if (name == "decel_x") {
                    decel_x_ = parameter.as_double();
                } else if (name == "min_vel_y") {
                    min_vel_y_ = parameter.as_double();
                } else if (name == "max_vel_y") {
                    max_vel_y_ = parameter.as_double();
                } else if (name == "acc_y") {
                    acc_y_ = parameter.as_double();
                } else if (name == "decel_y") {
                    decel_y_ = parameter.as_double();
                } else if (name == "min_vel_theta") {
                    min_vel_theta_ = parameter.as_double();
                } else if (name == "max_vel_theta") {
                    max_vel_theta_ = parameter.as_double();
                } else if (name == "acc_theta") {
                    acc_theta_ = parameter.as_double();
                } else if (name == "decel_theta") {
                    decel_theta_ = parameter.as_double();
                }
            }
        }
        result.successful = true;
        return result;
    }
};
}

#endif