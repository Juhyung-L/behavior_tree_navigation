#include <limits>

#include "nav2_util/geometry_utils.hpp"

#include "controller_server/controller_server.hpp"

namespace controller_server
{
ControllerServer::ControllerServer(const rclcpp::NodeOptions & options)
: nav2_util::LifecycleNode("controller_server", "", options)
, controller_loader_("gnc_core", "gnc_core::Controller")
, controller_type_("dwa_core::DWALocalPlanner")
, odom_topic_("odom")
, logger_(get_logger())
{
    RCLCPP_INFO(logger_, "Creating controller server");

    declare_parameter("controller_frequency", 20.0);
    declare_parameter("controller_type", 
        rclcpp::ParameterValue(std::string("dwa_core::DWALocalPlanner")));
    declare_parameter("odom_topic", rclcpp::ParameterValue(std::string("odom")));
    costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
        "local_costmap", std::string{get_namespace()}, "local_costmap");
}

ControllerServer::~ControllerServer()
{
    costmap_thread_.reset();
}

nav2_util::CallbackReturn
ControllerServer::on_configure(const rclcpp_lifecycle::State& /*state*/)
{
    RCLCPP_INFO(logger_, "Configuring");

    controller_frequency_ = get_parameter("controller_frequency").as_double();
    controller_type_ = get_parameter("controller_type").as_string();
    odom_topic_ = get_parameter("odom_topic").as_string();

    auto node = shared_from_this();

    costmap_ros_->configure();
    costmap_thread_ = std::make_unique<nav2_util::NodeThread>(costmap_ros_);
    odom_smoother_ = std::make_unique<nav2_util::OdomSmoother>(node, 0.3, odom_topic_);
    action_server_ = std::make_unique<ActionServer>(
        node,
        "follow_path",
        std::bind(&ControllerServer::executeController, this),
        nullptr,
        std::chrono::milliseconds(500),
        true);
    goal_checker_ = std::make_unique<GoalChecker>();
    progress_checker_ = std::make_unique<ProgressChecker>();
    goal_checker_->initialize(node);
    progress_checker_->initialize(node);

    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(
        "cmd_vel", rclcpp::SystemDefaultsQoS());
    
    controller_ = controller_loader_.createUniqueInstance(controller_type_);
    try
    {
        controller_->configure(node, costmap_ros_->getTfBuffer(), costmap_ros_);
    }
    catch (std::exception& ex)
    {
        RCLCPP_ERROR(logger_, 
            "Failed to load controller of type %s"
            "\nError message: %s",
            controller_type_.c_str(),
            ex.what());
        return nav2_util::CallbackReturn::FAILURE;
    }
    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn 
ControllerServer::on_activate(const rclcpp_lifecycle::State& /*state*/)
{
    RCLCPP_INFO(logger_, "Activating");

    costmap_ros_->activate();
    controller_->activate();
    cmd_vel_pub_->on_activate();
    action_server_->activate();

    createBond();
    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn 
ControllerServer::on_deactivate(const rclcpp_lifecycle::State& /*state*/)
{
    RCLCPP_INFO(logger_, "Deactivating");

    costmap_ros_->deactivate();
    controller_->deactivate();
    cmd_vel_pub_->on_deactivate();
    action_server_->deactivate();

    destroyBond();
    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn 
ControllerServer::on_cleanup(const rclcpp_lifecycle::State& /*state*/)
{
    RCLCPP_INFO(logger_, "Cleaning Up");

    costmap_ros_->cleanup();
    action_server_.reset();
    odom_smoother_.reset();
    costmap_thread_.reset();
    goal_checker_.reset();
    progress_checker_.reset();
    controller_.reset();
    cmd_vel_pub_.reset();
    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn 
ControllerServer::on_shutdown(const rclcpp_lifecycle::State& /*state*/)
{
    RCLCPP_INFO(logger_, "Shutting Down");
    return nav2_util::CallbackReturn::SUCCESS;
}

void ControllerServer::executeController()
{
    progress_checker_->reset();
    rclcpp::WallRate loop_rate(controller_frequency_);
    while (rclcpp::ok())
    {
        // cancel requested
        if (action_server_->is_cancel_requested())
        {
            action_server_->terminate_all();
            publishZeroVelocity();
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
            publishZeroVelocity();
            return;
        }

        // exit if robot hasn't progressed
        if (!progress_checker_->isProgressed(cur_pose.pose))
        {
            RCLCPP_ERROR(logger_, "Failed to progress towards goal");
            action_server_->terminate_current();
            publishZeroVelocity();
            return;
        }
        
        // last pose of global path is the goal
        const nav_msgs::msg::Path& global_path = action_server_->get_current_goal()->path;
        geometry_msgs::msg::Pose goal_pose = global_path.poses.back().pose;
        if (goal_checker_->isGoalReached(cur_pose.pose, goal_pose))
        {
            RCLCPP_INFO(logger_, "Goal reached");
            publishZeroVelocity();
            action_server_->succeeded_current();
            return;
        }
        geometry_msgs::msg::Twist cur_vel = odom_smoother_->getTwist();

        // compute velocity command
        geometry_msgs::msg::Twist cmd_vel = controller_->computeVelocityCommand(
            cur_pose.pose, cur_vel, global_path);
        cmd_vel_pub_->publish(std::move(cmd_vel));
        publishFeedback(cur_pose.pose, cmd_vel, global_path);

        if (!loop_rate.sleep())
        {
            RCLCPP_WARN(logger_, "Control loop missed its desired rate of %.4fHz",
                controller_frequency_);
        }
    }
}

void ControllerServer::publishFeedback(
    const geometry_msgs::msg::Pose& cur_pose,
    const geometry_msgs::msg::Twist& cmd_vel,
    const nav_msgs::msg::Path& global_path)
{
    std::shared_ptr<Action::Feedback> feedback = std::make_shared<Action::Feedback>();
    feedback->speed = std::hypot(cmd_vel.linear.x, cmd_vel.linear.y);

    size_t closest_pose_idx = 0;
    double cur_min_dist = std::numeric_limits<double>::max();
    for (size_t i=0; i<global_path.poses.size(); ++i)
    {
        double cur_dist = nav2_util::geometry_utils::euclidean_distance(
            cur_pose, global_path.poses[i].pose);
        if (cur_dist < cur_min_dist)
        {
            cur_min_dist = cur_dist;
            closest_pose_idx = i;
        }
    }
    feedback->distance_to_goal = nav2_util::geometry_utils::calculate_path_length(
        global_path, closest_pose_idx);
    action_server_->publish_feedback(feedback);
}

void ControllerServer::publishZeroVelocity()
{
    geometry_msgs::msg::Twist zero_vel;
    zero_vel.linear.x = 0.0;
    zero_vel.linear.y = 0.0;
    zero_vel.linear.z = 0.0;
    zero_vel.angular.x = 0.0;
    zero_vel.angular.y = 0.0;
    zero_vel.angular.z = 0.0;
    cmd_vel_pub_->publish(std::move(zero_vel));
}
}