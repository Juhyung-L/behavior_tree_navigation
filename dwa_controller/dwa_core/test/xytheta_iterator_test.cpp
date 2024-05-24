#include <gtest/gtest.h>

#include <string>
#include <fstream>
#include <vector>
#include <sstream>
#include <iostream>
#include <exception>

#include "rclcpp/rclcpp.hpp"
#include "nav2_util/lifecycle_node.hpp"

#include "dwa_core/xytheta_velocity_iterator.hpp"
#include "nav_2d_msgs/msg/twist2_d.hpp"

const double EPSILON{1E-5};
const std::vector<std::vector<double>> correct_twists{
    {-0.3, -0.3, -0.3},
    {0.0, -0.3, -0.3},
    {0.3, -0.3, -0.3},

    {-0.3, 0.0, -0.3},
    {0.0, 0.0, -0.3},
    {0.3, 0.0, -0.3},
    
    {-0.3, 0.3, -0.3},
    {0.0, 0.3, -0.3},
    {0.3, 0.3, -0.3},
    
    {-0.3, -0.3, 0.0},
    {0.0, -0.3, 0.0},
    {0.3, -0.3, 0.0},

    {-0.3, 0.0, 0.0},
    {0.0, 0.0, 0.0},
    {0.3, 0.0, 0.0},
    
    {-0.3, 0.3, 0.0},
    {0.0, 0.3, 0.0},
    {0.3, 0.3, 0.0},

    {-0.3, -0.3, 0.3},
    {0.0, -0.3, 0.3},
    {0.3, -0.3, 0.3},

    {-0.3, 0.0, 0.3},
    {0.0, 0.0, 0.3},
    {0.3, 0.0, 0.3},
    
    {-0.3, 0.3, 0.3},
    {0.0, 0.3, 0.3},
    {0.3, 0.3, 0.3}
};

void setParameters(nav2_util::LifecycleNode::SharedPtr nh)
{
    nh->set_parameter(rclcpp::Parameter("vx_samples", 3));
    nh->set_parameter(rclcpp::Parameter("vy_samples", 3));
    nh->set_parameter(rclcpp::Parameter("vtheta_samples", 3));
    nh->set_parameter(rclcpp::Parameter("min_vel_x", -0.3));
    nh->set_parameter(rclcpp::Parameter("max_vel_x", 0.3));
    nh->set_parameter(rclcpp::Parameter("decel_x", -2.5));
    nh->set_parameter(rclcpp::Parameter("acc_x", 2.5));
    nh->set_parameter(rclcpp::Parameter("min_vel_y", -0.3));
    nh->set_parameter(rclcpp::Parameter("max_vel_y", 0.3));
    nh->set_parameter(rclcpp::Parameter("decel_y", -2.5));
    nh->set_parameter(rclcpp::Parameter("acc_y", 2.5));
    nh->set_parameter(rclcpp::Parameter("min_vel_theta", -0.3));
    nh->set_parameter(rclcpp::Parameter("max_vel_theta", 0.3));
    nh->set_parameter(rclcpp::Parameter("decel_theta", -2.5));
    nh->set_parameter(rclcpp::Parameter("acc_theta", 2.5));
}

TEST(package_name, test1)
{
    std::string correct_twist_filename = "correct_twists.txt";

    rclcpp::NodeOptions node_options;
    auto nh = std::make_shared<nav2_util::LifecycleNode>("xytheta_iterator_test_node", "", node_options);
    
    dwa_core::KinematicsParameters::SharedPtr kp = std::make_shared<dwa_core::KinematicsParameters>(nh);
    dwa_core::XYThetaVelocityIterator vel_it(nh);
    setParameters(nh);
    kp->initialize();

    nav_2d_msgs::msg::Twist2D cur_vel;
    cur_vel.x = 0.0;
    cur_vel.y = 0.0;
    cur_vel.theta = 0.0;
    double look_ahead_time = 3.0;

    vel_it.initialize(cur_vel, look_ahead_time, kp);
    int twist_idx{0};
    nav_2d_msgs::msg::Twist2D twist;

    while (!vel_it.isFinished())
    {
        twist = vel_it.getCurrentVel();
        // std::cout << "x: " << twist.x << " y: " << twist.y << " theta: " << twist.theta << std::endl;
        EXPECT_NEAR(correct_twists[twist_idx][0], twist.x, EPSILON);
        EXPECT_NEAR(correct_twists[twist_idx][1], twist.y, EPSILON);
        EXPECT_NEAR(correct_twists[twist_idx][2], twist.theta, EPSILON);
        ++vel_it;
        ++twist_idx;
    }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}