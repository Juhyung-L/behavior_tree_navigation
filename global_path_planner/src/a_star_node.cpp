#include <algorithm>

#include "lifecycle_msgs/msg/state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "global_path_planner/a_star_node.hpp"

namespace a_star
{
AStar::AStar(const rclcpp::NodeOptions& options)
: nav2_util::LifecycleNode("a_star_node", "",options)
, logger_(this->get_logger())
{
    costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
        "global_costmap", std::string{get_namespace()}, "global_costmap"
    );
    path_.header.frame_id = "map";
}

AStar::~AStar()
{
    costmap_thread_.reset();
}

nav2_util::CallbackReturn
AStar::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
    RCLCPP_INFO(logger_, "Configuring");

    costmap_ros_->configure();
    costmap_ = costmap_ros_->getCostmap();
    costmap_thread_ = std::make_unique<nav2_util::NodeThread>(costmap_ros_);

    path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
        "plan", rclcpp::SystemDefaultsQoS()
    );
    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "goal_pose", rclcpp::SystemDefaultsQoS(), std::bind(&AStar::goalCB, this, _1)
    );

    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
AStar::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
    RCLCPP_INFO(logger_, "Activating");

    path_pub_->on_activate();
    costmap_ros_->activate();

    // call computePath at 2Hz
    timer_ = this->create_wall_timer(
        500ms, std::bind(&AStar::computePath, this)
    );

    createBond();
    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
AStar::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
    RCLCPP_INFO(logger_, "Deactivating");

    goal_received_ = false;
    path_pub_->on_deactivate();
    timer_->cancel();
    timer_.reset();
    timer_ = nullptr;
    if (costmap_ros_->get_current_state().id() ==
        lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
    {
        costmap_ros_->deactivate();
    }

    destroyBond();
    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
AStar::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
    RCLCPP_INFO(logger_, "Cleaning up");

    path_pub_.reset();
    goal_sub_.reset();

    node_pool_.clear();
    node_grid_.clear();
    prev_size_x_ = 0;
    prev_size_y_ = 0;

    if (costmap_ros_->get_current_state().id() ==
        lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
    {
        costmap_ros_->cleanup();
    }
    costmap_thread_.reset();
    costmap_ = nullptr;
    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
AStar::on_shutdown(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(logger_, "Shutting down");
    return nav2_util::CallbackReturn::SUCCESS;
}

void AStar::goalCB(const geometry_msgs::msg::PoseStamped goal)
{
    if (this->get_current_state().id() !=
        lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
    {
        return;
    }
    unsigned int goal_x, goal_y;
    costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, goal_x, goal_y);
    goal_.x = static_cast<int>(goal_x);
    goal_.y = static_cast<int>(goal_y);
    goal_received_ = true;
}

void AStar::computePath()
{
    if (!goal_received_)
    {
        return;
    }

    // lock the costmap for the duration of computing the path
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap_->getMutex()));

    //get the start position
    geometry_msgs::msg::PoseStamped robot_pose;
    if (!costmap_ros_->getRobotPose(robot_pose))
    {
        RCLCPP_ERROR(logger_, "Could not get robot pose");
        return;
    }
    unsigned int src_x, src_y;
    costmap_->worldToMap(robot_pose.pose.position.x, robot_pose.pose.position.y, src_x, src_y);
    src_.x = static_cast<int>(src_x);
    src_.y = static_cast<int>(src_y);

    resetNodes();
    q_ = std::priority_queue<AStarNode*, std::vector<AStarNode*>, cmp>();

    // create the start node
    double g = getTraversalCost(src_.x, src_.y);
    double h = getHCost(src_.x, src_.y);
    *node_pool_it_ = AStarNode{src_.x, src_.y, g, h, g+h, nullptr, true};
    addNodeToGrid(src_.x, src_.y, &(*node_pool_it_));
    q_.push(&(*node_pool_it_));
    ++node_pool_it_;
    
    AStarNode* cur_node;
    while (!q_.empty())
    {
        cur_node = q_.top();
        q_.pop();
        cur_node->in_queue = false;

        // reached goal or reached unknown cell
        if ((cur_node->x == goal_.x && cur_node->y == goal_.y) ||
            costmap_->getCost(cur_node->x, cur_node->y) == nav2_costmap_2d::NO_INFORMATION)
        {
            break;
        }

        getNeighbors(cur_node);
    }

    if (q_.empty())
    {
        // if exited while loop and queue is empty,
        // that means path could not be found
        return;
    }

    // backtrace to get the path
    backtrace(cur_node);

    path_.header.stamp = this->now();
    path_pub_->publish(path_);
}

void AStar::resetNodes()
{
    size_x_ = static_cast<int>(costmap_->getSizeInCellsX());
    size_y_ = static_cast<int>(costmap_->getSizeInCellsY());

    // reserve more space only if map size grew
    if (size_x_ > prev_size_x_ || size_y_ > prev_size_y_)
    {
        node_grid_.reserve(size_x_ * size_y_);
        node_pool_.reserve(size_x_ * size_y_);
        int size_inc = (size_x_ - prev_size_x_) * (size_y_ - prev_size_y_);
        for (int i=0; i<size_inc; ++i)
        {
            node_pool_.emplace_back(); // push default constructed AStarNodes
            node_grid_.push_back(nullptr);
        }
        for (int i=0; i<prev_size_x_*prev_size_y_; ++i)
        {
            node_grid_[i] = nullptr;
        }
    }
    else
    {
        for (int i=0; i<size_x_*size_y_; ++i)
        {
            node_grid_[i] = nullptr;
        }
    }
    
    node_pool_it_ = node_pool_.begin();
    prev_size_x_ = size_x_;
    prev_size_y_ = size_y_;
}

void AStar::getNeighbors(AStarNode* cur_node)
{
    AStarNode* new_node;
    int new_x, new_y;
    double g;

    for (auto direction : directions)
    {
        new_x = cur_node->x + direction[0];
        new_y = cur_node->y + direction[1];

        // check if new coordinate is inside map and obstacle-free
        if ((0 <= new_x && new_x < size_x_ && 0 <= new_y && new_y < size_y_) &&
            costmap_->getCost(new_x, new_y) != nav2_costmap_2d::LETHAL_OBSTACLE)
        {
            new_node = getNode(new_x, new_y);
            if (new_node == nullptr)
            {
                *node_pool_it_ = AStarNode();
                node_pool_it_->x = new_x;
                node_pool_it_->y = new_y;
                node_pool_it_->h = getHCost(new_x, new_y);
                addNodeToGrid(new_x, new_y, &(*node_pool_it_));
                new_node = getNode(new_x, new_y);
                ++node_pool_it_;
            }

            g = cur_node->g + std::hypot(cur_node->x - new_x, cur_node->y - new_y) 
                + getTraversalCost(new_x, new_y);

            // if found shorter path to node from start
            // update g and f and push it to queue if not already in queue
            if (new_node->g > g)
            {
                new_node->g = g;
                new_node->f = g + new_node->h;
                new_node->parent = cur_node;
                if (!new_node->in_queue)
                {
                    q_.push(new_node);
                    new_node->in_queue = true;
                }
            }
        }
    }
}

void AStar::backtrace(AStarNode* cur_node)
{
    path_.poses.clear();
    geometry_msgs::msg::PoseStamped pose;
    while (!(cur_node->x == src_.x && cur_node->y == src_.y))
    {
        costmap_->mapToWorld(cur_node->x, cur_node->y,
            pose.pose.position.x, pose.pose.position.y
        );
        path_.poses.push_back(pose);
        cur_node = cur_node->parent;
    }
    // add the start pose to path
    costmap_->mapToWorld(cur_node->x, cur_node->y,
        pose.pose.position.x, pose.pose.position.y
    );
    path_.poses.push_back(pose);
    std::reverse(path_.poses.begin(), path_.poses.end());
}
}