#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/utils/shared_library.h"

#include "bt_navigators/bt_engine.hpp"

namespace bt_navigators
{
BTEngine::BTEngine(const std::vector<std::string>& plugin_libraries)
{
    BT::SharedLibrary loader;
    for (const auto & p : plugin_libraries)
    {
        factory_.registerFromPlugin(loader.getOSName(p));
    }
}

BTStatus BTEngine::run(
    BT::Tree* tree,
    std::function<void()> onLoop,
    std::function<bool()> cancelRequested,
    std::chrono::milliseconds loop_timeout)
{
    rclcpp::WallRate loopRate(loop_timeout);
    BT::NodeStatus result = BT::NodeStatus::RUNNING;

    // loop until NodeStatus is not RUNNING or something happens with ROS
    // BtNodeStatus can be RUNNING, SUCCESS, FAILURE, or IDLE
    // so if it's not RUNNING, we can stop the loop
    try
    {
        while (rclcpp::ok() && result == BT::NodeStatus::RUNNING)
        {
            if (cancelRequested()) 
            {
                tree->haltTree();
                return BTStatus::CANCELED;
            }

            result = tree->tickOnce();
            onLoop();

            if (!loopRate.sleep()) 
            {
                RCLCPP_WARN(
                rclcpp::get_logger("BTEngine"),
                    "Behavior Tree tick rate %0.2f was exceeded!",
                    1.0 / (loopRate.period().count() * 1.0e-9));
            }
        }
    }
    catch(const std::exception& ex)
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("BTEngine"),
            "Behavior tree threw exception: %s. Exiting with failure.", ex.what());
        return BTStatus::FAILED;
    }

    // if it reached this point, that means the tree did not get canceled in the middle of execution
    // so it's either SUCCEEDED or FAILED
    return (result == BT::NodeStatus::SUCCESS) ? BTStatus::SUCCEEDED : BTStatus::FAILED;
}

BT::Tree
BTEngine::createTreeFromText(
    const std::string& xml_string,
    BT::Blackboard::Ptr blackboard)
{
    return factory_.createTreeFromText(xml_string, blackboard);
}

BT::Tree
BTEngine::createTreeFromFile(
    const std::string& file_path,
    BT::Blackboard::Ptr blackboard)
{
    return factory_.createTreeFromFile(file_path, blackboard);
}

void
BTEngine::haltAllActions(BT::Tree& tree)
{
    tree.haltTree();
}
}