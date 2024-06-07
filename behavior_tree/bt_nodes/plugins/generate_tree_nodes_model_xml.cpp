#include <string>
#include <fstream>
#include <iostream>

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/xml_parsing.h"
#include "behaviortree_cpp/action_node.h"

#include "bt_nodes/plugins/action/compute_path_to_pose_action.hpp"
#include "bt_nodes/plugins/action/follow_path_action.hpp"
#include "bt_nodes/plugins/control/pipeline_sequence.hpp"
#include "bt_nodes/plugins/control/recovery.hpp"
#include "bt_nodes/plugins/control/round_robin.hpp"

#define REGISTER_ACTION_NODE(factory, node_class, pascal_node_name) \
do \
{ \
    BT::NodeBuilder builder = \
        [](const std::string& name, const BT::NodeConfiguration& config) \
        { \
            return std::make_unique<node_class>(name, name, config); \
        }; \
    factory.registerBuilder<node_class>(pascal_node_name, builder); \
} \
while(0) \

#define REGISTER_CONTROL_NODE(factory, node_class, pascal_node_name) \
do \
{ \
    BT::NodeBuilder builder = \
        [](const std::string& name, const BT::NodeConfiguration& config) \
        { \
            return std::make_unique<node_class>(name, config); \
        }; \
    factory.registerBuilder<node_class>(pascal_node_name, builder); \
} \
while(0) \

int main(int argc, char** argv)
{
    if (argc != 2)
    {
        std::cerr << "Missing tree nodes xml filename\n";
        return -1;
    }

    // open file
    std::string filename = argv[1];
    std::ofstream file(filename);
    if (!file.is_open()) 
    {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return -1;
    }

    // generate xml string
    BT::BehaviorTreeFactory factory;
    REGISTER_ACTION_NODE(factory, bt_nodes::ComputePathToPoseAction, "ComputePathToPose");
    REGISTER_ACTION_NODE(factory, bt_nodes::FollowPathAction, "FollowPath");
    REGISTER_CONTROL_NODE(factory, bt_nodes::Recovery, "Recovery");
    REGISTER_CONTROL_NODE(factory, bt_nodes::RoundRobin, "RoundRobin");
    REGISTER_CONTROL_NODE(factory, bt_nodes::PipelineSequence, "PipelineSequence");

    std::string xml_str = BT::writeTreeNodesModelXML(factory);

    // save into file
    
    file << xml_str;
    if (file.fail()) 
    {
        std::cerr << "Failed to write to file: " << filename << std::endl;
        return -1;
    }
    file.close();

    return 0;
}