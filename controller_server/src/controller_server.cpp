#include "controller_server/controller_server.hpp"

namespace controller_server
{
ControllerServer::ControllerServer(const rclcpp::NodeOptions & options)
: nav2_util::LifecycleNode("controller_server", "", options)
, controller_loader_("gnc_core", "gnc_core::Controller")
, controller_type_("dwa_core::DWALocalPlanner")
, logger_(get_logger())
{
    RCLCPP_INFO(logger_, "Creating controller server");

    declare_parameter("controller_frequency", 20.0);
    declare_parameter("controller_type", 
        rclcpp::ParameterValue(std::string("dwa_core::DWALocalPlanner")));

    costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
        "local_costmap", std::string{get_namespace()}, "local_costmap");
}

ControllerServer::~ControllerServer()
{
    controller_.reset();
    costmap_thread_.reset();
}

nav2_util::CallbackReturn
ControllerServer::on_configure(const rclcpp_lifecycle::State& /*state*/)
{
    
}
}