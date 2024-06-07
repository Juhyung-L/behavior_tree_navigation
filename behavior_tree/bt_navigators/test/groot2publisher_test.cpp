#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include "behaviortree_cpp/blackboard.h"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/utils/shared_library.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"

// WARNING: this test needs to be run with planner_server and controller_server

const std::vector<std::string> plugin_libraries{
    "follow_path_action_bt_node",
    "compute_path_to_pose_action_bt_node",
    "pipeline_sequence_bt_node",
    "recovery_bt_node",
    "round_robin_bt_node"};

class GrootTestFixture : public ::testing::Test
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
        config_->blackboard->set<std::chrono::milliseconds>("wait_for_service_timeout", std::chrono::milliseconds(100));
        config_->blackboard->set<bool>("initial_pose_received", false);
    
        BT::SharedLibrary loader;
        for (const auto& p : plugin_libraries)
        {
            factory_->registerFromPlugin(loader.getOSName(p));
        }
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
        groot_pub_.reset();
    }

    static rclcpp::Node::SharedPtr node_;
    static BT::NodeConfiguration* config_;
    static std::shared_ptr<BT::BehaviorTreeFactory> factory_;
    std::shared_ptr<BT::Tree> tree_;
    std::shared_ptr<BT::Groot2Publisher> groot_pub_;
};

rclcpp::Node::SharedPtr GrootTestFixture::node_ = nullptr;
BT::NodeConfiguration* GrootTestFixture::config_ = nullptr;
std::shared_ptr<BT::BehaviorTreeFactory> GrootTestFixture::factory_ = nullptr;

TEST_F(GrootTestFixture, test)
{
    std::string file_path = ament_index_cpp::get_package_share_directory("bt_navigators") +
        "/behavior_trees/navigate_to_pose.xml";
    tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromFile(file_path, config_->blackboard));
    groot_pub_ = std::make_shared<BT::Groot2Publisher>(*tree_);
    rclcpp::spin(node_); 
    EXPECT_TRUE(true);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}