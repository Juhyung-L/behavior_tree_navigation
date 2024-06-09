#ifndef BASE_CRITIC_HPP_
#define BASE_CRITIC_HPP_

#include <memory>
#include <string>

#include "geometry_msgs/msg/pose2_d.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_util/lifecycle_node.hpp"

#include "nav_2d_msgs/msg/path2_d.hpp"

namespace dwa_critics
{
    /**
     * @class BaseCritic
     * @brief Abstract class that all critics inherit from
    */
class BaseCritic
{
public:
    using Ptr = std::shared_ptr<BaseCritic>;
    virtual ~BaseCritic() = default;
    void initialize(
        nav2_util::LifecycleNode::WeakPtr parent,
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros,
        const std::string& plugin_name)
    {
        parent_ = parent;
        costmap_ros_ = costmap_ros;
        costmap_ = costmap_ros->getCostmap();
        plugin_name_ = plugin_name;
        on_initialize();

        auto node = parent.lock();
        RCLCPP_INFO(node->get_logger(), "%s Critic loaded", critic_name_.c_str());
    }

    virtual void reset() {}
    /**
     * @param global_path Global path inside of the local costmap
    */
    virtual void prepare(const nav_2d_msgs::msg::Path2D& global_path) = 0;
    virtual double scorePath(const nav_2d_msgs::msg::Path2D& local_path) = 0;
    double weight_;
    bool invert_score_;

protected:
    virtual void on_initialize() = 0;
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
    nav2_costmap_2d::Costmap2D* costmap_;
    nav2_util::LifecycleNode::WeakPtr parent_;
    std::string critic_name_;
    std::string plugin_name_;
};
}

#endif