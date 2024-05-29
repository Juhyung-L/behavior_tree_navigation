#include <memory>

#include "gtest/gtest.h"
#include "nav2_util/lifecycle_node.hpp"
#include "geometry_msgs/msg/pose.hpp"

#include "controller_server/goal_checker.hpp"

class TestLifecycleNode : public nav2_util::LifecycleNode
{
public:
    explicit TestLifecycleNode(const std::string& name)
    : nav2_util::LifecycleNode(name)
    {}

    nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State&)
    {
        return nav2_util::CallbackReturn::SUCCESS;
    }

    nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State&)
    {
        return nav2_util::CallbackReturn::SUCCESS;
    }

    nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State&)
    {
        return nav2_util::CallbackReturn::SUCCESS;
    }

    nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State&)
    {
        return nav2_util::CallbackReturn::SUCCESS;
    }

    nav2_util::CallbackReturn onShutdown(const rclcpp_lifecycle::State&)
    {
        return nav2_util::CallbackReturn::SUCCESS;
    }

    nav2_util::CallbackReturn onError(const rclcpp_lifecycle::State&)
    {
        return nav2_util::CallbackReturn::SUCCESS;
    }
};

TEST(ControllerServer, goal_checker_test)
{
    auto node = std::make_shared<TestLifecycleNode>("dummy_node");
    controller_server::GoalChecker goal_checker = controller_server::GoalChecker();
    goal_checker.initialize(node);

    geometry_msgs::msg::Pose cur_pose;
    geometry_msgs::msg::Pose goal_pose;
    
    // default xy_tolerance=0.25
    // default yaw_tolerance=0.25

    // distance test
    cur_pose.position.x = 0.0;
    cur_pose.position.y = 0.0;
    cur_pose.position.z = 0.0;
    cur_pose.orientation.x = 0.0;
    cur_pose.orientation.y = 0.0;
    cur_pose.orientation.z = 0.0;
    cur_pose.orientation.w = 1.0;
    goal_pose.position.x = 0.24;
    goal_pose.position.y = 0.0;
    goal_pose.position.z = 0.0;
    goal_pose.orientation.x = 0.0;
    goal_pose.orientation.y = 0.0;
    goal_pose.orientation.z = 0.0;
    goal_pose.orientation.w = 1.0;
    EXPECT_TRUE(goal_checker.isGoalReached(cur_pose, goal_pose));
    cur_pose.position.x = 0.0;
    cur_pose.position.y = 0.0;
    cur_pose.position.z = 0.0;
    cur_pose.orientation.x = 0.0;
    cur_pose.orientation.y = 0.0;
    cur_pose.orientation.z = 0.0;
    cur_pose.orientation.w = 1.0;
    goal_pose.position.x = 0.26;
    goal_pose.position.y = 0.0;
    goal_pose.position.z = 0.0;
    goal_pose.orientation.x = 0.0;
    goal_pose.orientation.y = 0.0;
    goal_pose.orientation.z = 0.0;
    goal_pose.orientation.w = 1.0;
    EXPECT_FALSE(goal_checker.isGoalReached(cur_pose, goal_pose));

    // angle test
    cur_pose.position.x = 0.0;
    cur_pose.position.y = 0.0;
    cur_pose.position.z = 0.0;
    cur_pose.orientation.x = 0.0;
    cur_pose.orientation.y = 0.0;
    cur_pose.orientation.z = 0.0;
    cur_pose.orientation.w = 1.0;
    goal_pose.position.x = 0.0;
    goal_pose.position.y = 0.0;
    goal_pose.position.z = 0.0;
    // RPY rotation: x=0.0 y=0.0 z=0.24
    goal_pose.orientation.x = 0.0;
    goal_pose.orientation.y = 0.0;
    goal_pose.orientation.z = 0.1197122;
    goal_pose.orientation.w = 0.9928086;
    EXPECT_TRUE(goal_checker.isGoalReached(cur_pose, goal_pose));
    cur_pose.position.x = 0.0;
    cur_pose.position.y = 0.0;
    cur_pose.position.z = 0.0;
    cur_pose.orientation.x = 0.0;
    cur_pose.orientation.y = 0.0;
    cur_pose.orientation.z = 0.0;
    cur_pose.orientation.w = 1.0;
    goal_pose.position.x = 0.0;
    goal_pose.position.y = 0.0;
    goal_pose.position.z = 0.0;
    // RPY rotation: x=0.0 y=0.0 z=0.26
    goal_pose.orientation.x = 0.0;
    goal_pose.orientation.y = 0.0;
    goal_pose.orientation.z = 0.1296341;
    goal_pose.orientation.w = 0.9915619;
    EXPECT_FALSE(goal_checker.isGoalReached(cur_pose, goal_pose));
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}