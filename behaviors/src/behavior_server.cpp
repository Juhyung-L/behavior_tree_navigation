#include "tf2_ros/create_timer_ros.h"

#include "behaviors/behavior_server.hpp"

namespace behavior_server
{
BehaviorServer::BehaviorServer(const rclcpp::NodeOptions& options)
: nav2_util::LifecycleNode("behavior_server", "", options)
, plugin_loader_("gnc_core", "gnc_core::Behavior")
{
    declare_parameter("costmap_topic", "local_costmap/costmap_raw");
    declare_parameter("footprint_topic", "local_costmap/published_footprint");
    declare_parameter("cycle_frequency", 10.0);
    declare_parameter("costmap_frame", "odom");
    declare_parameter("robot_frame", "base_footprint");
    declare_parameter("transform_tolerance", 0.1);
    declare_parameter("behavior_types", std::vector<std::string>{"BackUp", "Wait"});
}

BehaviorServer::~BehaviorServer()
{}

nav2_util::CallbackReturn BehaviorServer::on_configure(const rclcpp_lifecycle::State& /*state*/)
{
    RCLCPP_INFO(logger_, "Configuring");

    tf_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    get_node_base_interface(),
    get_node_timers_interface());
    tf_->setCreateTimerInterface(timer_interface);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_);

    std::string costmap_topic = get_parameter("costmap_topic").as_string();
    std::string footprint_topic = get_parameter("footprint_topic").as_string();
    std::string robot_frame = get_parameter("robot_frame").as_string();
    double transform_tolerance = get_parameter("transform_tolerance").as_double();
    
    costmap_sub_ = std::make_shared<nav2_costmap_2d::CostmapSubscriber>(
        shared_from_this(), costmap_topic);
    footprint_sub_ = std::make_unique<nav2_costmap_2d::FootprintSubscriber>(
        shared_from_this(), footprint_topic, *tf_, robot_frame, transform_tolerance);
    collision_checker_ = std::make_shared<nav2_costmap_2d::CostmapTopicCollisionChecker>(
        *costmap_sub_, *footprint_sub_);

    if (!loadBehaviors())
    {
        return nav2_util::CallbackReturn::FAILURE;
    }
    return nav2_util::CallbackReturn::SUCCESS;
}

bool BehaviorServer::loadBehaviors()
{    
    std::vector<std::string> behavior_types = get_parameter("behavior_types").as_string_array();
    for (size_t i=0; i<behavior_types.size(); ++i)
    {
        std::string behavior_type = "behaviors::" + behavior_types[i];
        try
        {
            behaviors_.push_back(plugin_loader_.createUniqueInstance(behavior_type));
            behaviors_[i]->configure(shared_from_this(), tf_, collision_checker_, costmap_sub_);
        }
        catch (std::exception& ex)
        {
            RCLCPP_ERROR(logger_, "Failed to load behavior of type %s", behavior_type.c_str());
            return false;
        }
    }
    return true;
}

nav2_util::CallbackReturn BehaviorServer::on_activate(const rclcpp_lifecycle::State& /*state*/)
{
    RCLCPP_INFO(logger_, "Activating");

    for (auto& behvaior : behaviors_)
    {
        behvaior->activate();
    }

    createBond();
    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn BehaviorServer::on_deactivate(const rclcpp_lifecycle::State& /*state*/)
{
    RCLCPP_INFO(logger_, "Deactivating");

    for (auto& behvaior : behaviors_)
    {
        behvaior->deactivate();
    }

    destroyBond();
    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn BehaviorServer::on_cleanup(const rclcpp_lifecycle::State& /*state*/)
{
    RCLCPP_INFO(logger_, "Cleaning up");

    behaviors_.clear();
    tf_listener_.reset();
    tf_.reset();

    costmap_sub_.reset();
    footprint_sub_.reset();
    collision_checker_.reset();
    costmap_sub_.reset();
    footprint_sub_.reset();

    for (auto& behvaior : behaviors_)
    {
        behvaior->cleanup();
    }

    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn BehaviorServer::on_shutdown(const rclcpp_lifecycle::State& /*state*/)
{
    RCLCPP_INFO(logger_, "Shutting down");
    return nav2_util::CallbackReturn::SUCCESS;
}
}