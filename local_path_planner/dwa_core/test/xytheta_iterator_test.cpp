#include <gtest/gtest.h>

#include "rclcpp/rclcpp.hpp"
#include "nav2_util/lifecycle_node.hpp"

#include "dwa_core/xytheta_velocity_iterator.hpp"
#include "nav_2d_msgs/msg/twist2_d.hpp"

const double EPSILON{1E-5};

struct Twist2D
{
    double x; 
    double y;
    double theta;

    Twist2D(double x, double y, double theta)
    : x(x), y(y), theta(theta)
    {}
};

std::vector<Twist2D> getCorrectTwists()
{
    std::vector<Twist2D> twists;
    twists.emplace_back(0.0, 0.0, -1.0);
    twists.emplace_back(1.0, 0.0, -1.0);
    twists.emplace_back(2.0, 0.0, -1.0);
    twists.emplace_back(3.0, 0.0, -1.0);

    twists.emplace_back(0.0, 0.0, -0.33333);
    twists.emplace_back(1.0, 0.0, -0.33333);
    twists.emplace_back(2.0, 0.0, -0.33333);
    twists.emplace_back(3.0, 0.0, -0.33333);

    twists.emplace_back(0.0, 0.0, 0.33333);
    twists.emplace_back(1.0, 0.0, 0.33333);
    twists.emplace_back(2.0, 0.0, 0.33333);
    twists.emplace_back(3.0, 0.0, 0.33333);

    twists.emplace_back(0.0, 0.0, 1.0);
    twists.emplace_back(1.0, 0.0, 1.0);
    twists.emplace_back(2.0, 0.0, 1.0);
    twists.emplace_back(3.0, 0.0, 1.0);
    return twists;
}

std::vector<rclcpp::Parameter> setKinematicsParameters()
{
    std::vector<rclcpp::Parameter> parameters;
    parameters.push_back(rclcpp::Parameter("min_vel_x", 0.0));
    parameters.push_back(rclcpp::Parameter("max_vel_x", 3.0));
    parameters.push_back(rclcpp::Parameter("acc_x", 1.0));
    parameters.push_back(rclcpp::Parameter("decel_x", -1.0));

    parameters.push_back(rclcpp::Parameter("min_vel_y", 0.0));
    parameters.push_back(rclcpp::Parameter("max_vel_y", 0.0));
    parameters.push_back(rclcpp::Parameter("acc_y", 0.0));
    parameters.push_back(rclcpp::Parameter("decel_y", 0.0));

    parameters.push_back(rclcpp::Parameter("min_vel_theta", -1.0));
    parameters.push_back(rclcpp::Parameter("max_vel_theta", 1.0));
    parameters.push_back(rclcpp::Parameter("acc_theta", 0.5));
    parameters.push_back(rclcpp::Parameter("decel_theta", -0.5));
    return parameters;
}

TEST(package_name, test1)
{
    rclcpp::NodeOptions node_options;
    node_options.parameter_overrides(setKinematicsParameters());
    auto nh = std::make_shared<nav2_util::LifecycleNode>("dummy_test_node", "", node_options);
    
    dwa_core::KinematicsParameters::SharedPtr kp = std::make_shared<dwa_core::KinematicsParameters>(nh);
    int num_terations{3};
    dwa_core::XYThetaVelocityIterator vel_it(num_terations, kp);

    nav_2d_msgs::msg::Twist2D cur_vel;
    cur_vel.x = 0.0;
    cur_vel.y = 0.0;
    cur_vel.theta = 0.0;
    double look_ahead_time = 3.0;

    vel_it.initialize(cur_vel, look_ahead_time);
    std::vector<Twist2D> correct_twists = getCorrectTwists();
    int twist_idx{0};
    nav_2d_msgs::msg::Twist2D twist;

    while (!vel_it.isFinished())
    {
        twist = vel_it.getCurrentVel();
        EXPECT_NEAR(correct_twists[twist_idx].x, twist.x, EPSILON);
        EXPECT_NEAR(correct_twists[twist_idx].y, twist.y, EPSILON);
        EXPECT_NEAR(correct_twists[twist_idx].theta, twist.theta, EPSILON);
        ++vel_it;
        twist_idx++;
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