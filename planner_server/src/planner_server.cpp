#include "planner_server/planner_server.hpp"

namespace planner_server
{
PlannerServer::PlannerServer(const rclcpp::NodeOptions& options)
: nav2_util::LifecycleNode("planner_server", "", options)
, planner_loader_("gnc_core", "gnc_core::Planner")
, planner_type_("AStarSearchGlobalPlanner")
, logger_(get_logger())
{
    RCLCPP_INFO(logger_, "Creating planner server");

    declare_parameter("planner_type", "a_star_planner::AStarGlobalPlanner");

    costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
        "global_costmap", std::string{get_namespace()}, "global_costmap");
}

PlannerServer::~PlannerServer()
{
    costmap_thread_.reset();
}

nav2_util::CallbackReturn
PlannerServer::on_configure(const rclcpp_lifecycle::State& /*state*/)
{
    RCLCPP_INFO(logger_, "Configuring");

    planner_type_ = get_parameter("planner_type").as_string();

    auto node = shared_from_this();

    costmap_ros_->configure();
    costmap_thread_ = std::make_unique<nav2_util::NodeThread>(costmap_ros_);
    action_server_ = std::make_unique<ActionServer>(
        node,
        "compute_path_to_pose",
        std::bind(&PlannerServer::executePlanner, this),
        nullptr,
        std::chrono::milliseconds(500),
        true);
    path_pub_ = create_publisher<nav_msgs::msg::Path>(
        "global_path", rclcpp::SystemDefaultsQoS());

    planner_ = planner_loader_.createUniqueInstance(planner_type_);
    try
    {
        planner_->configure(node, tf_, costmap_ros_);
    }
    catch (std::exception& ex)
    {
        RCLCPP_ERROR(logger_, 
            "Failed to load planner of type %s"
            "\n Error message: %s",
            planner_type_.c_str(),
            ex.what());
        return nav2_util::CallbackReturn::FAILURE;
    }
    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn 
PlannerServer::on_activate(const rclcpp_lifecycle::State& /*state*/)
{
    RCLCPP_INFO(logger_, "Activating");

    costmap_ros_->activate();
    planner_->activate();
    path_pub_->is_activated();
    action_server_->activate();

    createBond();
    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn 
PlannerServer::on_deactivate(const rclcpp_lifecycle::State& /*state*/)
{
    RCLCPP_INFO(logger_, "Deactivating");

    costmap_ros_->deactivate();
    planner_->deactivate();
    path_pub_->on_deactivate();
    action_server_->deactivate();

    destroyBond();
    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn 
PlannerServer::on_cleanup(const rclcpp_lifecycle::State& /*state*/)
{
    RCLCPP_INFO(logger_, "Cleaning up");

    costmap_ros_->cleanup();
    action_server_.reset();
    costmap_thread_.reset();
    planner_.reset();
    path_pub_.reset();
    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn 
PlannerServer::on_shutdown(const rclcpp_lifecycle::State& /*state*/)
{
    RCLCPP_INFO(logger_, "Shutting Down");
    return nav2_util::CallbackReturn::SUCCESS;
}

void PlannerServer::executePlanner()
{
    auto goal = action_server_->get_current_goal();
    auto result = std::make_shared<gnc_msgs::action::ComputePathToPose::Result>();
    auto start_time = this->now();

    // cancel requested
    if (action_server_->is_cancel_requested())
    {
        action_server_->terminate_all();
        return;
    }

    // if there is preempt goal, accept it immediately
    if (action_server_->is_preempt_requested())
    {
        action_server_->accept_pending_goal();
    }

    // wait for costmap to become valid (after clearing costmap)
    rclcpp::Rate r(100);
    while (!costmap_ros_->isCurrent())
    {
        r.sleep();
    }

    geometry_msgs::msg::PoseStamped cur_pose;
    if (!costmap_ros_->getRobotPose(cur_pose))
    {
        RCLCPP_ERROR(logger_, "Failed to get robot pose");
        action_server_->terminate_current();
        return;
    }

    // make sure the path isn't empty (which means the planner was unable to find a path)
    nav_msgs::msg::Path path = planner_->computePath(cur_pose, goal->goal);
    if (path.poses.empty())
    {
        RCLCPP_ERROR(logger_, "Failed to compute path");
        action_server_->terminate_current();
        return;
    }

    result->path = path;
    result->planning_time = this->now() - start_time;
    path_pub_->publish(result->path);
    action_server_->succeeded_current(result);
}
}