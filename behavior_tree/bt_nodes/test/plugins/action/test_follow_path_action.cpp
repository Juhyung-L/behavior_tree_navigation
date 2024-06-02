#include <gtest/gtest.h>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "behaviortree_cpp/bt_factory.h"

#include "bt_nodes/plugins/action/follow_path_action.hpp"

// WARNING: THIS TEST IS MEANT TO BE RUN WITH GAZEBO SIMULATION

// these paths are specific to the map used for simulation
std::vector<std::vector<double>> path1 = {
    {-1.0, 1.0},
    {-1.0, 1.2},
    {-1.0, 1.4},
    {-1.0, 1.6},
    {-1.0, 1.8},
    {-1.0, 2.0}
};

std::vector<std::vector<double>> path2 = {
    {-1.0, 2.0},
    {-1.2, 1.8},
    {-1.4, 1.6},
    {-1.6, 1.4},
    {-1.8, 1.2},
    {-2.0, 1.0}
};

std::vector<std::vector<double>> path3 = {
    {-1.5, 1.5},
    {-1.3, 1.3},
    {-1.1, 1.1},
    {-1.0, 1.0}
};

class FollowPathActionTestFixture : public ::testing::Test
{
public:
    static void SetUpTestCase()
    {
        node_ = std::make_shared<rclcpp::Node>("follow_path_action_test_fixture");
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
                return std::make_unique<bt_nodes::FollowPathAction>(
                    name, "follow_path", config);
            };
        factory_->registerBuilder<bt_nodes::FollowPathAction>("FollowPath", builder);
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
    static std::shared_ptr<BT::Tree> tree_;
};

rclcpp::Node::SharedPtr FollowPathActionTestFixture::node_ = nullptr;
BT::NodeConfiguration* FollowPathActionTestFixture::config_ = nullptr;
std::shared_ptr<BT::BehaviorTreeFactory> FollowPathActionTestFixture::factory_ = nullptr;
std::shared_ptr<BT::Tree> FollowPathActionTestFixture::tree_ = nullptr;

TEST_F(FollowPathActionTestFixture, test_tick)
{
    std::string xml_txt =
        R"(
        <root BTCPP_format="4" >
            <BehaviorTree ID="MainTree">
                <FollowPath name="FollowPath" path="{path}"/>
            </BehaviorTree>
        </root>)";
    tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));

    rclcpp::WallRate r(20.0);
    nav_msgs::msg::Path path;
    path.header.frame_id = "map";
    
    // follow path1
    path.poses.resize(path1.size());
    for (size_t i=0; i<path1.size(); ++i)
    {
        path.poses[i].pose.position.x = path1[i][0];
        path.poses[i].pose.position.y = path1[i][1];
    }
    config_->blackboard->set<nav_msgs::msg::Path>("path", path);
    while (rclcpp::ok() && tree_->rootNode()->status() != BT::NodeStatus::SUCCESS)
    {
        tree_->rootNode()->executeTick();
        r.sleep();
    }

    tree_->haltTree();

    // change to path3 while following path2
    path.poses.resize(path2.size());
    for (size_t i=0; i<path2.size(); ++i)
    {
        path.poses[i].pose.position.x = path2[i][0];
        path.poses[i].pose.position.y = path2[i][1];
    }
    config_->blackboard->set<nav_msgs::msg::Path>("path", path);
    rclcpp::Clock clock = rclcpp::Clock();
    rclcpp::Duration timeout = rclcpp::Duration::from_seconds(3.0);
    rclcpp::Time start = clock.now();
    while (rclcpp::ok() && (clock.now() - start < timeout))
    {
        tree_->rootNode()->executeTick();
        r.sleep();
    }

    path.poses.resize(path3.size());
    for (size_t i=0; i<path3.size(); ++i)
    {
        path.poses[i].pose.position.x = path3[i][0];
        path.poses[i].pose.position.y = path3[i][1];
    }
    config_->blackboard->set<nav_msgs::msg::Path>("path", path);
    while (rclcpp::ok() && tree_->rootNode()->status() != BT::NodeStatus::SUCCESS)
    {
        tree_->rootNode()->executeTick();
        r.sleep();
    }

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
