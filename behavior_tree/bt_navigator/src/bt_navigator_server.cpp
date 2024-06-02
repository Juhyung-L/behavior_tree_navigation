#include "bt_navigator/bt_navigator_server.hpp"

/**
 * ROS parameters declared
 * - global_frame
 * - robot_frame
 * - transform_tolerance
 * - odom_topic
 * - plugin_lib_names
*/

namespace bt_navigator
{
BTNavigatorServer::BTNavigatorServer(rclcpp::NodeOptions options)
: nav2_util::LifecycleNode("bt_navigator", "", options)
, class_loader_("gnc_core", "gnc_core::BTNavigatorBase")
{
    declare_parameter("global_frame", "map");
    declare_parameter("robot_frame", "base_link");
    declare_parameter("transform_tolerance", 0.1);
    declare_parameter("odom_topic", "odom");
    declare_parameter("plugin_lib_names", rclcpp::ParameterValue(std::vector<std::string>{}));
    declare_parameter("navigator_type", "bt_navigator::NavigateToPoseNavigator");
}

BTNavigatorServer::~BTNavigatorServer()
{}

nav2_util::CallbackReturn
BTNavigatorServer::on_configure(const rclcpp_lifecycle::State& /*state*/)
{
    RCLCPP_INFO(get_logger(), "Configuring");

    std::string global_frame = get_parameter("global_frame").as_string();
    std::string robot_frame = get_parameter("robot_frame").as_string();
    double transform_tolerance = get_parameter("transform_tolerance").as_double();
    std::string odom_topic = get_parameter("odom_topic").as_string();

    tf_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
        get_node_base_interface(), get_node_timers_interface());
    tf_->setCreateTimerInterface(timer_interface);
    tf_->setUsingDedicatedThread(true);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_, this, false);

    std::vector<std::string> plugin_lib_names = get_parameter("plugin_lib_names").as_string_array();

    auto node = shared_from_this();
    odom_smoother_ = std::make_shared<nav2_util::OdomSmoother>(node, 0.3, odom_topic);

    std::string navigator_type = get_parameter("navigator_type").as_string();
    navigator_ = class_loader_.createUniqueInstance(navigator_type);
    try
    {
        if (!navigator_->on_configure(node, odom_smoother_, plugin_lib_names, tf_, robot_frame, global_frame, transform_tolerance))
        {
            return nav2_util::CallbackReturn::FAILURE;
        }
    }
    catch (std::exception& e)
    {
        RCLCPP_ERROR(get_logger(), 
            "Failed to configure navigator of type %s"
            "\nException: %s",
            navigator_type.c_str(),
            e.what());
    }
    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
BTNavigatorServer::on_activate(const rclcpp_lifecycle::State& /*state*/)
{
    RCLCPP_INFO(get_logger(), "Activating");

    if (!navigator_->on_activate())
    {
        return nav2_util::CallbackReturn::FAILURE;
    }

    createBond();
    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
BTNavigatorServer::on_deactivate(const rclcpp_lifecycle::State& /*state*/)
{
    RCLCPP_INFO(get_logger(), "Deactivating");
    if (!navigator_->on_deactivate())
    {
        return nav2_util::CallbackReturn::FAILURE;
    }

    destroyBond();
    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
BTNavigatorServer::on_cleanup(const rclcpp_lifecycle::State& /*state*/)
{
    RCLCPP_INFO(get_logger(), "Cleaning up");
    tf_listener_.reset();
    tf_.reset();
    odom_smoother_.reset();
    navigator_.reset();
    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
BTNavigatorServer::on_shutdown(const rclcpp_lifecycle::State& /*state*/)
{
    RCLCPP_INFO(get_logger(), "Shutting down");
    return nav2_util::CallbackReturn::SUCCESS;
}
}