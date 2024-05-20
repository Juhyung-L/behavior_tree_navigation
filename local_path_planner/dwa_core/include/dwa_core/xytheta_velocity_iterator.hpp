#ifndef XYTHETA_VELOCITY_ITERATOR_HPP_
#define XYTHETA_VELOCITY_ITERATOR_HPP_

#include "rclcpp/rclcpp.hpp"

#include "nav_2d_msgs/msg/twist2_d.hpp"
#include "dwa_core/one_d_velocity_iterator.hpp"
#include "dwa_core/kinematics_parameters.hpp"

namespace dwa_core
{
/**
 * @class XYThetaVelocityIterator
 * @brief A wrapper class for 3 OneDVelocityIterators (x, y, theta)
*/

class XYThetaVelocityIterator
{
public:
    XYThetaVelocityIterator()
    {}

    XYThetaVelocityIterator(nav2_util::LifecycleNode::WeakPtr parent)
    : parent_(parent)
    {
        auto node = parent_.lock();
        node->declare_parameter("vx_samples", rclcpp::ParameterValue(1));
        node->declare_parameter("vy_samples", rclcpp::ParameterValue(1));
        node->declare_parameter("vtheta_samples", rclcpp::ParameterValue(1));
    }

    void initialize(const nav_2d_msgs::msg::Twist2D& current_vel, double dt, const KinematicsParameters::SharedPtr kp)
    {
        auto node = parent_.lock();

        is_finished_ = false;
        x_it_.initialize(current_vel.x, kp->getMinVelX(), kp->getMaxVelX(),
            kp->getAccX(), kp->getDecelX(), dt, node->get_parameter("vx_samples").as_int()
        );
        y_it_.initialize(current_vel.y, kp->getMinVelY(), kp->getMaxVelY(),
            kp->getAccY(), kp->getDecelY(), dt, node->get_parameter("vy_samples").as_int()
        );
        theta_it_.initialize(current_vel.theta, kp->getMinVelTheta(), kp->getMaxVelTheta(),
            kp->getAccTheta(), kp->getDecelTheta(), dt, node->get_parameter("vtheta_samples").as_int()
        );
    }

    XYThetaVelocityIterator& operator++()
    {
        if (!x_it_.isFinished())
        {
            ++x_it_;
            return *this;
        }

        if (!y_it_.isFinished())
        {
            x_it_.reset();
            ++y_it_;
            return *this;
        }

        if (!theta_it_.isFinished())
        {
            x_it_.reset();
            y_it_.reset();
            ++theta_it_;
            return *this;
        }

        is_finished_ = true;
        return *this;
    }

    bool isFinished()
    {
        return is_finished_;
    }

    nav_2d_msgs::msg::Twist2D getCurrentVel()
    {
        nav_2d_msgs::msg::Twist2D twist;
        twist.x = x_it_.getCurrentVelocity();
        twist.y = y_it_.getCurrentVelocity();
        twist.theta = theta_it_.getCurrentVelocity();
        return twist;
    }

private:
    bool is_finished_;

    rclcpp_lifecycle::LifecycleNode::WeakPtr parent_;

    OneDVelocityIterator x_it_;
    OneDVelocityIterator y_it_;
    OneDVelocityIterator theta_it_;
};
}

#endif