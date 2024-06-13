#ifndef BEHAVIOR_HPP_
#define BEHAVIOR_HPP_

#include <memory>

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"
#include "nav2_costmap_2d/costmap_topic_collision_checker.hpp"
#include "nav2_costmap_2d/costmap_subscriber.hpp"

namespace gnc_core
{
class Behavior
{
public:
    using Ptr = std::shared_ptr<Behavior>;

    virtual ~Behavior() = default;

    virtual void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr parent,
        std::shared_ptr<tf2_ros::Buffer> tf,
        std::shared_ptr<nav2_costmap_2d::CostmapTopicCollisionChecker> collision_checker,
        std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub) = 0;

    virtual void activate() = 0;
    virtual void deactivate() = 0;
    virtual void cleanup() = 0;
};
}

#endif