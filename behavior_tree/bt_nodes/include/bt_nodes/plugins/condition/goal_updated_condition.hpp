#ifndef GOAL_UPDATED_CONDITION_HPP_
#define GOAL_UPDATED_CONDITION_HPP_

#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "behaviortree_cpp/condition_node.h"

namespace bt_nodes
{
class GoalUpdatedCondition : public BT::ConditionNode
{
public:
    GoalUpdatedCondition(
        const std::string& condition_name,
        const BT::NodeConfiguration& conf);

    BT::NodeStatus tick() override;
    static BT::PortsList providedPorts() {return {};}
private:
    geometry_msgs::msg::PoseStamped prev_goal_;
};
}

#endif