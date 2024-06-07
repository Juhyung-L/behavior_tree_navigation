#include <gtest/gtest.h>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "behaviortree_cpp/bt_factory.h"

#include "bt_nodes/plugins/action/compute_path_to_pose_action.hpp"

// WARNING: THIS TEST IS MEANT TO BE RUN WITH GAZEBO SIMULATION

class ComputePathToPoseTestFixture : public ::testing::Test
{
public:
    static void SetUpTestCase()
    {
        node_ = std::make_shared<rclcpp::Node>("compute_path_to_pose_action_test_fixture");
        factory_ = std::make_shared<BT::BehaviorTreeFactory>();
        config_ = new BT::NodeConfiguration();
        config_->blackboard = BT::Blackboard::create();
        config_->blackboard->set<rclcpp::Node::SharedPtr>("node", node_);
        config_->blackboard->set<std::chrono::milliseconds>("server_timeout", std::chrono::milliseconds(20));
        config_->blackboard->set<std::chrono::milliseconds>("bt_loop_duration", std::chrono::milliseconds(10));
        config_->blackboard->set<std::chrono::milliseconds>("wait_for_service_timeout", std::chrono::milliseconds(1000));
        config_->blackboard->set<bool>("initial_pose_received", false);
        
        BT::NodeBuilder builder =
            [](const std::string& name, const BT::NodeConfiguration& config)
            {
                return std::make_unique<bt_nodes::ComputePathToPoseAction>(
                    name, "compute_path_to_pose", config);
            };
        factory_->registerBuilder<bt_nodes::ComputePathToPoseAction>("ComputePathToPose", builder);
    }

    static void TearDownTestCase()
    {
        delete config_;
        config_ = nullptr;
        node_.reset();
        factory_.reset();
    }

    void TearDown() override
    {
        tree_.reset();
    }

    static rclcpp::Node::SharedPtr node_;
    static BT::NodeConfiguration* config_;
    static std::shared_ptr<BT::BehaviorTreeFactory> factory_;
    std::shared_ptr<BT::Tree> tree_;
};

rclcpp::Node::SharedPtr ComputePathToPoseTestFixture::node_ = nullptr;
BT::NodeConfiguration* ComputePathToPoseTestFixture::config_ = nullptr;
std::shared_ptr<BT::BehaviorTreeFactory> ComputePathToPoseTestFixture::factory_ = nullptr;

TEST_F(ComputePathToPoseTestFixture, test_tick)
{
    std::string xml_txt = 
        R"(
        <root BTCPP_format="4" >
            <BehaviorTree ID="MainTree">
                <ComputePathToPose name="ComputePathToPose" goal="{goal}"/>
            </BehaviorTree>
        </root>)";
    tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));

    rclcpp::WallRate r(20.0);
    BT::NodeStatus status = BT::NodeStatus::IDLE;
    geometry_msgs::msg::PoseStamped goal_pose;
    goal_pose.header.frame_id = "map";

    // this pose is reachable
    goal_pose.pose.position.x = -2.0;
    goal_pose.pose.position.y = 1.0;
    goal_pose.pose.position.z = 0.0;
    goal_pose.pose.orientation.x = 0.0;
    goal_pose.pose.orientation.y = 0.0;
    goal_pose.pose.orientation.z = 0.0;
    goal_pose.pose.orientation.w = 1.0;
    config_->blackboard->set<geometry_msgs::msg::PoseStamped>("goal", goal_pose);
    while (rclcpp::ok() && status != BT::NodeStatus::SUCCESS)
    {
        tree_->rootNode()->executeTick();
        status = tree_->rootNode()->status();
        r.sleep();
    }
    tree_->haltTree();

    // this pose is not reachable
    goal_pose.pose.position.x = 0.0;
    goal_pose.pose.position.y = -1.0;
    goal_pose.pose.position.z = 0.0;
    goal_pose.pose.orientation.x = 0.0;
    goal_pose.pose.orientation.y = 0.0;
    goal_pose.pose.orientation.z = 0.0;
    goal_pose.pose.orientation.w = 1.0;
    config_->blackboard->set<geometry_msgs::msg::PoseStamped>("goal", goal_pose);
    while (rclcpp::ok() && status != BT::NodeStatus::FAILURE)
    {
        tree_->rootNode()->executeTick();
        status = tree_->rootNode()->status();
        r.sleep();
    }
    tree_->haltTree();

    EXPECT_TRUE(true);
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    rclcpp::init(argc, argv);
    int all_successful = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return all_successful;
}