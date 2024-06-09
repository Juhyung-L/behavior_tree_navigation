#ifndef COMPUTE_PATH_TO_POSE_HPP_
#define COMPUTE_PATH_TO_POSE_HPP_

#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

#include "bt_nodes/bt_action_node.hpp"
#include "gnc_msgs/action/compute_path_to_pose.hpp"

namespace bt_nodes
{
class ComputePathToPoseAction : public BTActionNode<gnc_msgs::action::ComputePathToPose>
{
    using Action = gnc_msgs::action::ComputePathToPose;
    using ActionResult = Action::Result;
public:
    ComputePathToPoseAction(
        const std::string& xml_tag_name,
        const std::string& action_name,
        const BT::NodeConfiguration& conf);

    void on_tick() override;
    BT::NodeStatus on_success() override;
    BT::NodeStatus on_aborted() override;
    BT::NodeStatus on_cancelled() override;
    void halt() override;
    void on_wait_for_result(
        std::shared_ptr<const gnc_msgs::action::ComputePathToPose::Feedback> feedback) override;
    static BT::PortsList providedPorts()
    {
        return providedBasicPorts({
            BT::InputPort<geometry_msgs::msg::PoseStamped>("goal", "Destination to plan to"),
            BT::OutputPort<nav_msgs::msg::Path>("path", "Path created by ComputePathToPose action")
        });
    }
};
}

#endif