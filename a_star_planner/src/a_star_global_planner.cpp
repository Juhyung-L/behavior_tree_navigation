#include <algorithm>
#include <string>

#include "a_star_planner/a_star_global_planner.hpp"

namespace a_star_planner
{
AStarGlobalPlanner::AStarGlobalPlanner()
: gnc_core::Planner()
{}

AStarGlobalPlanner::~AStarGlobalPlanner()
{}

void AStarGlobalPlanner::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent,
    std::shared_ptr<tf2_ros::Buffer> /*tf*/,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
    auto node = parent.lock();

    logger_ = node->get_logger();
    clock_ = node->get_clock();

    costmap_ros_ = costmap_ros;    
    costmap_ros_->configure();
    costmap_ = costmap_ros_->getCostmap();
}

void AStarGlobalPlanner::activate()
{}

void AStarGlobalPlanner::deactivate()
{}

void AStarGlobalPlanner::cleanup()
{
    node_pool_.clear();
    node_grid_.clear();
}

nav_msgs::msg::Path AStarGlobalPlanner::computePath(
    const geometry_msgs::msg::PoseStamped& start,
    const geometry_msgs::msg::PoseStamped& goal)
{
    // lock the costmap for the duration of computing the path
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap_->getMutex()));

    // convert start and goal to map coordinates
    unsigned int start_x, start_y;
    costmap_->worldToMap(start.pose.position.x, start.pose.position.y, start_x, start_y);
    start_m_.x = static_cast<int>(start_x);
    start_m_.y = static_cast<int>(start_y);

    unsigned int goal_x, goal_y;
    costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, goal_x, goal_y);
    goal_m_.x = static_cast<int>(goal_x);
    goal_m_.y = static_cast<int>(goal_y);

    resetNodes();
    q_ = std::priority_queue<AStarNode*, std::vector<AStarNode*>, cmp>();

    // create the start node
    double g = getTraversalCost(start_m_.x, start_m_.y);
    double h = getHCost(start_m_.x, start_m_.y);
    *node_pool_it_ = AStarNode{start_m_.x, start_m_.y, g, h, g+h, nullptr, true};
    addNodeToGrid(start_m_.x, start_m_.y, &(*node_pool_it_));
    q_.push(&(*node_pool_it_));
    ++node_pool_it_;
    
    AStarNode* cur_node;
    while (!q_.empty())
    {
        cur_node = q_.top();
        q_.pop();
        cur_node->in_queue = false;

        // reached goal or reached unknown cell
        if ((cur_node->x == goal_m_.x && cur_node->y == goal_m_.y) ||
            costmap_->getCost(cur_node->x, cur_node->y) == nav2_costmap_2d::NO_INFORMATION)
        {
            break;
        }

        getNeighbors(cur_node);
    }

    if (!q_.empty())
    {
        // if exited while loop and queue is empty, that means path could not be found
        // only backtrace when a path is found
        backtrace(cur_node);
    }
    path_.header.stamp = clock_->now();
    return path_;
}

void AStarGlobalPlanner::resetNodes()
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

void AStarGlobalPlanner::getNeighbors(AStarNode* cur_node)
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

void AStarGlobalPlanner::backtrace(AStarNode* cur_node)
{
    path_.poses.clear();
    geometry_msgs::msg::PoseStamped pose;
    while (!(cur_node->x == start_m_.x && cur_node->y == start_m_.y))
    {
        // transform map coordinates to world coordinates
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

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(a_star_planner::AStarGlobalPlanner, gnc_core::Planner)