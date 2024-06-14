#include <gtest/gtest.h>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/bt_factory.h"

#include "bt_nodes/plugins/action/back_up_action.hpp"

// WARNING: THIS TEST IS MEANT TO BE RUN WITH GAZEBO SIMULATION

class BackUpTestFixture : public ::testing::Test
{
public:
    static void SetUpTestCase()
    {
        node_ = std::make_shared<rclcpp::Node>("back_up_action_test_fixture");
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
                return std::make_unique<bt_nodes::BackUpAction>(
                    name, "back_up", config);
            };
        factory_->registerBuilder<bt_nodes::BackUpAction>("BackUp", builder);
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

rclcpp::Node::SharedPtr BackUpTestFixture::node_ = nullptr;
BT::NodeConfiguration* BackUpTestFixture::config_ = nullptr;
std::shared_ptr<BT::BehaviorTreeFactory> BackUpTestFixture::factory_ = nullptr;

TEST_F(BackUpTestFixture, test_tick)
{
    std::string xml_txt = 
        R"(
        <root BTCPP_format="4" >
            <BehaviorTree ID="MainTree">
                <BackUp back_up_dist="0.3" back_up_speed="0.3" back_up_duration="5"/>
            </BehaviorTree>
        </root>)";
    tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));


    rclcpp::WallRate r(20.0);
    BT::NodeStatus status = BT::NodeStatus::IDLE;
    rclcpp::Time start_time = node_->now();
    while (rclcpp::ok() && status != BT::NodeStatus::SUCCESS)
    {
        tree_->rootNode()->executeTick();
        status = tree_->rootNode()->status();
        r.sleep();
    }
    rclcpp::Duration elapsed_time = node_->now() - start_time;
    tree_->haltTree();

    EXPECT_NEAR(elapsed_time.seconds(), 3.0, 1.0);
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    rclcpp::init(argc, argv);
    int all_successful = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return all_successful;
}