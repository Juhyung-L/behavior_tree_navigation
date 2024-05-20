#ifndef BT_ENGINE_HPP_
#define BT_ENGINE_HPP_

#include <vector>

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/xml_parsing.h"

namespace bt_interface
{
enum class BTStatus {SUCCEEDED, FAILED, CANCELED};

class BTEngine
{
public:
    explicit BTEngine(const std::vector<std::string>& plugin_libraries);

    BTStatus run(
        BT::Tree* tree,
        std::function<void()> onLoop,
        std::function<bool()> cancelRequested,
        std::chrono::milliseconds loop_timeout = std::chrono::milliseconds(10));
    
    BT::Tree createTreeFromText(
        const std::string& xml_string,
        BT::Blackboard::Ptr blackboard);

    BT::Tree createTreeFromFile(
        const std::string& file_path,
        BT::Blackboard::Ptr blackboard);

    void haltAllActions(BT::Tree& tree);

private:
    BT::BehaviorTreeFactory factory_;
};
}

#endif