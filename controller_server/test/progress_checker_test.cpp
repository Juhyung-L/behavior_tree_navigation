#include <memory>
#include <chrono>

#include "gtest/gtest.h"
#include "nav2_util/lifecycle_node.hpp"
#include "geometry_msgs/msg/pose.hpp"

#include "controller_server/progress_checker.hpp"

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

TEST(ControllerServer, progress_checker_test)
{
    auto node = std::make_shared<TestLifecycleNode>("dummy_node");
    controller_server::ProgressChecker progress_checker = controller_server::ProgressChecker();
    progress_checker.initialize(node);

    // default minimum_distance=0.5 
    // default movement_time_tolerance=10(seconds)
    geometry_msgs::msg::Pose cur_pose;
    cur_pose.position.x = 0.0;
    cur_pose.position.y = 0.0;
    cur_pose.position.z = 0.0;
    cur_pose.orientation.x = 0.0;
    cur_pose.orientation.y = 0.0;
    cur_pose.orientation.z = 0.0;
    cur_pose.orientation.w = 1.0;

    // first pose should return true
    EXPECT_TRUE(progress_checker.isProgressed(cur_pose));

    cur_pose.position.x = 0.51;
    EXPECT_TRUE(progress_checker.isProgressed(cur_pose));

    std::chrono::duration<double> sleep_duration = 
        std::chrono::duration<double>(5.1);
    std::this_thread::sleep_for(sleep_duration);

    EXPECT_FALSE(progress_checker.isProgressed(cur_pose));   
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}