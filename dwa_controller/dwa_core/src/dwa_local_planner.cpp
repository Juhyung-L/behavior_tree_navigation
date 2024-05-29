#include <chrono>
#include <vector>
#include <limits>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "nav2_util/node_utils.hpp"

#include "dwa_core/dwa_local_planner.hpp"
#include "dwa_util/conversions.hpp"
#include "dwa_util/transform_utils.hpp"
#include "nav_2d_msgs/msg/twist2_d.hpp"
#include "dwa_critics/base_critic.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace dwa_core
{
/**
 * @struct Node
 * @brief Simple struct to encapsulate velocity command and its associated critic scores
*/
struct Node
{
    nav_2d_msgs::msg::Twist2D cmd_vel;
    std::vector<double> critics_scores;
    double total_score{0.0};
};
/**
 * @struct DebugNode
 * @brief Used to send path information to RViz plugin for visualization
*/
struct DebugNode
{
    nav_2d_msgs::msg::Path2D path;
    double total_score;
};

DWALocalPlanner::DWALocalPlanner()
: gnc_core::Controller()
, critic_loader_("dwa_critics", "dwa_critics::BaseCritic")
, plugin_name_("DWALocalPlanner")
{}

void
DWALocalPlanner::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
    auto node = parent.lock();

    nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".sim_time", rclcpp::ParameterValue(1.5));
    nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".time_granularity", rclcpp::ParameterValue(0.1));
    nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".debug", rclcpp::ParameterValue(false));
    nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".critic_names", 
        rclcpp::ParameterValue(std::vector<std::string>{"GlobalPathAlign", "ObstacleProximity", "Homing"}));

    sim_time_ = node->get_parameter(plugin_name_ + ".sim_time").as_double();
    time_granularity_ = node->get_parameter(plugin_name_ + ".time_granularity").as_double();
    debug_ = node->get_parameter(plugin_name_ + ".debug").as_bool();
    critic_names_ = node->get_parameter(plugin_name_ + ".critic_names").as_string_array();

    steps_ = std::ceil(sim_time_ / time_granularity_);
    global_path_set_ = false;
    prev_num_vel_samples_ = 0;

    costmap_ros_ = costmap_ros;
    costmap_ = costmap_ros_->getCostmap();
    
    // debug publishers (only publish if debug_ flag is true)
    // publishes sampled paths for visualization in RViz
    paths_pub_ = node->create_publisher<nav_2d_msgs::msg::DWATrajectories>(
        "sample_paths", rclcpp::SystemDefaultsQoS());
    // publishes global path after it is processed by the prepareGlobalPath() function
    adjusted_global_path_pub_ = node->create_publisher<nav_msgs::msg::Path>(
        "adjusted_global_path", rclcpp::SystemDefaultsQoS());

    kp_ = std::make_shared<KinematicsParameters>(parent, plugin_name_);
    vel_it_ = XYThetaVelocityIterator(parent, plugin_name_);
    tf_buffer_ = tf;
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    if (!loadCritics(parent))
    {
        throw std::runtime_error("Failed to load DWA critics");
    }
}

DWALocalPlanner::~DWALocalPlanner()
{}

void DWALocalPlanner::activate()
{
    kp_->initialize();
    paths_pub_->on_activate();
    adjusted_global_path_pub_->on_activate();
}

void DWALocalPlanner::deactivate()
{
    global_path_set_ = false;
    paths_pub_->on_deactivate();
    adjusted_global_path_pub_->on_deactivate();
}

void DWALocalPlanner::cleanup()
{
    kp_.reset();
    paths_pub_.reset();
    adjusted_global_path_pub_.reset();
    tf_listener_.reset();
    for (auto& critic : critics_)
    {
        critic.reset();
    }
    costmap_ = nullptr;
}

geometry_msgs::msg::Twist DWALocalPlanner::computeVelocityCommand(
    const geometry_msgs::msg::Pose& current_pose,
    const geometry_msgs::msg::Twist& current_velocity,
    const nav_msgs::msg::Path& global_path)
{
    geometry_msgs::msg::Twist cmd_vel; // velocity to return
    if (!global_path_set_)
    {
        return cmd_vel;
    }
    
    // lock costmap
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap_->getMutex()));

    // prepare global path
    nav_2d_msgs::msg::Path2D global_path_2d = dwa_util::path3Dto2D(global_path);
    nav_2d_msgs::msg::Path2D adjusted_global_path = prepareGlobalPath(global_path_2d);
    nav_2d_msgs::msg::Path2D transformed_global_path;
    if (!dwa_util::transformPath2D(tf_buffer_, costmap_ros_->getBaseFrameID(), adjusted_global_path, transformed_global_path))
    {
        return cmd_vel; // failed to transform path
    }
    transformed_global_path.header.frame_id = costmap_ros_->getBaseFrameID();

    geometry_msgs::msg::Pose2D goal_pose;
    goal_pose.x = global_path_2d.poses.back().x;
    goal_pose.y = global_path_2d.poses.back().y;
    for (auto& critic : critics_)
    {
        critic->prepare(transformed_global_path, goal_pose);
    }

    if (debug_)
    {
        adjusted_global_path_pub_->publish(dwa_util::path2Dto3D(transformed_global_path));
    }

    geometry_msgs::msg::Pose2D current_pose_2d = dwa_util::pose3Dto2D(current_pose);
    nav_2d_msgs::msg::Twist2D current_vel_2d = dwa_util::twist3Dto2D(current_velocity);

    // sample local paths and score them
    vel_it_.initialize(current_vel_2d, sim_time_, kp_);
    std::vector<Node> nodes;
    nodes.reserve(prev_num_vel_samples_);
    int i = 0;
    std::vector<int> best_score_idxs{0, 0, 0};
    std::vector<int> worst_score_idxs{0, 0, 0};
    std::vector<DebugNode> debug_nodes;
    debug_nodes.reserve(prev_num_vel_samples_);
    while (!vel_it_.isFinished())
    {
        Node node;

        // generate paths
        nav_2d_msgs::msg::Twist2D cmd_vel = vel_it_.getCurrentVel();
        nav_2d_msgs::msg::Path2D path = generatePath(current_pose_2d, current_vel_2d, cmd_vel);
        node.cmd_vel = cmd_vel;

        // store paths for rviz
        if (debug_)
        {
            DebugNode debug_node;
            debug_node.path = path;
            debug_nodes.push_back(debug_node);
        }
        
        // score paths
        for (size_t j=0; j<critics_.size(); ++j)
        {
            node.critics_scores.push_back(critics_[j]->scorePath(path));            
        }
        nodes.push_back(node);

        for (size_t j=0; j<critics_.size(); ++j)
        {
            // store best score
            if (nodes[i].critics_scores[j] > nodes[best_score_idxs[j]].critics_scores[j])
            {
                best_score_idxs[j] = i;
            }

            // store worst score
            if (nodes[i].critics_scores[j] < nodes[worst_score_idxs[j]].critics_scores[j])
            {
                worst_score_idxs[j] = i;
            }
        }

        // increment velocity iterator
        ++vel_it_;
        ++i;
    }
    prev_num_vel_samples_ = i;

    // normalize scores
    for (size_t j=0; j<critics_.size(); ++j)
    {
        double best_score = nodes[best_score_idxs[j]].critics_scores[j];
        double worst_score = nodes[worst_score_idxs[j]].critics_scores[j];
        double diff = best_score - worst_score;
        if (diff < EPSILON)
        {
            // if the difference between the max and min scores is very small,
            // dont incorporate the critic into the total score
            continue;
        }

        for (int k=0; k<i; ++k)
        {
            double normalized_score;
            if (critics_[j]->invert_score_)
            {
                normalized_score = critics_[j]->weight_  * (1.0 - ((nodes[k].critics_scores[j] - worst_score) / diff));
            }
            else
            {
                normalized_score = critics_[j]->weight_ * (nodes[k].critics_scores[j] - worst_score) / diff;
            }
            nodes[k].total_score += normalized_score;
        }
    }

    int best_total_score_idx=0;
    for (int j=0; j<i; ++j)
    {
        if (nodes[j].total_score > nodes[best_total_score_idx].total_score)
        {
            best_total_score_idx = j;
        }
    }
    cmd_vel.linear.x = nodes[best_total_score_idx].cmd_vel.x;
    cmd_vel.linear.y = nodes[best_total_score_idx].cmd_vel.y;
    cmd_vel.angular.z = nodes[best_total_score_idx].cmd_vel.theta;

    if (debug_)
    {
        for (int j=0; j<i; ++j)
        {
            debug_nodes[j].total_score = nodes[j].total_score;
        }
        auto cmp = 
            [](const DebugNode& n1, const DebugNode& n2)
            {
                return n1.total_score > n2.total_score;
            };
        std::sort(debug_nodes.begin(), debug_nodes.end(), cmp);
        nav_2d_msgs::msg::DWATrajectories paths;
        paths.header.frame_id = costmap_ros_->getBaseFrameID();
        
        int num_paths_to_print = 10;
        paths.scores.reserve(num_paths_to_print);
        paths.paths.reserve(num_paths_to_print);
        for (int j=0; j<num_paths_to_print; ++j)
        {
            paths.scores.push_back(debug_nodes[j].total_score);
            paths.paths.push_back(debug_nodes[j].path);
        }
        paths_pub_->publish(paths);
    }

    return cmd_vel;
}

nav_2d_msgs::msg::Path2D DWALocalPlanner::generatePath(
    const geometry_msgs::msg::Pose2D& start_pose,
    const nav_2d_msgs::msg::Twist2D& current_vel,
    const nav_2d_msgs::msg::Twist2D& target_vel)
{
    nav_2d_msgs::msg::Path2D path;
    path.poses.push_back(start_pose); // push the start pose
    geometry_msgs::msg::Pose2D pose = start_pose;
    nav_2d_msgs::msg::Twist2D vel = current_vel;
    for (int i=1; i<steps_; ++i)
    {
        vel = computeVelocity(vel, target_vel, time_granularity_);
        pose = computePose(pose, vel, time_granularity_);
        path.poses.push_back(pose);
    }
    return path;
}

geometry_msgs::msg::Pose2D DWALocalPlanner::computePose(
    const geometry_msgs::msg::Pose2D& start_pose,
    const nav_2d_msgs::msg::Twist2D& vel,
    double dt)
{
    geometry_msgs::msg::Pose2D new_pose;
    new_pose.x = start_pose.x + (vel.x * cos(start_pose.theta) - vel.y * sin(start_pose.theta)) * dt;
    new_pose.y = start_pose.y + (vel.x * sin(start_pose.theta) + vel.y * cos(start_pose.theta)) * dt;
    new_pose.theta = start_pose.theta + vel.theta * dt;
    return new_pose;
}

nav_2d_msgs::msg::Twist2D DWALocalPlanner::computeVelocity(
    const nav_2d_msgs::msg::Twist2D& current_vel, 
    const nav_2d_msgs::msg::Twist2D& target_vel,
    double dt)
{
    nav_2d_msgs::msg::Twist2D new_vel;
    new_vel.x = projectVelocity(current_vel.x, kp_->getAccX(), kp_->getDecelX(), dt, target_vel.x);
    new_vel.y = projectVelocity(current_vel.y, kp_->getAccY(), kp_->getDecelY(), dt, target_vel.y);
    new_vel.theta = projectVelocity(current_vel.theta, kp_->getAccTheta(), kp_->getDecelTheta(), dt, target_vel.theta);
    return new_vel;
}

double DWALocalPlanner::projectVelocity(double current_vel, double acc, double decel, double dt, double target_vel)
{
    if (current_vel < target_vel)
    {
        current_vel = current_vel + acc * dt;
        return std::min(current_vel, target_vel);
    }
    else
    {
        current_vel = current_vel + decel * dt;
        return std::max(current_vel, target_vel);
    }
}

nav_2d_msgs::msg::Path2D DWALocalPlanner::prepareGlobalPath(nav_2d_msgs::msg::Path2D in_path)
{
    nav_2d_msgs::msg::Path2D out_path;
    out_path.header.frame_id = in_path.header.frame_id;

    if (in_path.poses.empty())
    {
        return out_path;
    }

    // cut the global path where the local costmap ends
    // if the global path ends inside of the costmap, it doesn't get cut
    unsigned int x, y;
    size_t i = 0;
    for (; i<in_path.poses.size(); ++i)
    {
        // the first pose that is outside of the local costmap
        if (!costmap_->worldToMap(in_path.poses[i].x, in_path.poses[i].y, x, y))
        {
            break;
        }
    }
    in_path.poses.resize(i);

    // interpolate
    geometry_msgs::msg::Pose2D prev = in_path.poses[0];
    out_path.poses.push_back(prev);
    double resolution = costmap_->getResolution();
    for (size_t i=0; i<in_path.poses.size(); ++i)
    {
        geometry_msgs::msg::Pose2D cur;
        cur.x = in_path.poses[i].x;
        cur.y = in_path.poses[i].y;
        double dx = (cur.x - prev.x);
        double dy = (cur.y - prev.y);
        double dtheta = (cur.theta - prev.theta);
        double dist = 
            std::sqrt(dx*dx + dy*dy);
        if (dist > resolution)
        {
            int steps = static_cast<int>(dist / resolution);
            double step_x = dx / steps;
            double step_y = dy / steps;
            double step_theta = dtheta / steps;
            for (int j=1; j<steps; ++j)
            {
                geometry_msgs::msg::Pose2D interpolated_pose;
                interpolated_pose.x = prev.x + j* step_x;
                interpolated_pose.y = prev.y + j* step_y;
                interpolated_pose.theta = step_theta + j * step_theta;
                out_path.poses.push_back(interpolated_pose);
            }
        }
        out_path.poses.push_back(cur);
        prev = cur;
    }

    // when cutting the global path, the last pose is the first pose outside of the costmap
    // then we interpolated
    // so we need to cut the path once more to get rid of interpolated poses outside of the costmap
    i = 0;
    for (; i<out_path.poses.size(); ++i)
    {
        if (!costmap_->worldToMap(out_path.poses[i].x, out_path.poses[i].y, x, y))
        {
            break;
        }
    }
    out_path.poses.resize(i);
    return out_path;
}

bool DWALocalPlanner::loadCritics(const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent)
{
    if (critic_names_.empty())
    {
        RCLCPP_ERROR(logger_, "No critics defined");
        return false;
    }

    for (auto critic_name : critic_names_)
    {
        critic_name = "dwa_critics::" + critic_name + "Critic";
        dwa_critics::BaseCritic::Ptr critic = critic_loader_.createUniqueInstance(critic_name);
        try
        {
            critic->initialize(parent, costmap_ros_, plugin_name_);
        }
        catch(const std::exception& e)
        {
            RCLCPP_ERROR(logger_, "Critic %s failed to initialize", critic_name.c_str());
            return false;
        }
        critics_.push_back(critic);
    }
    return true;
}
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(dwa_core::DWALocalPlanner, gnc_core::Controller)