#ifndef A_STAR_GLOBAL_PLANNER_HPP_
#define A_STAR_GLOBAL_PLANNER_HPP_

#include <memory>
#include <chrono>
#include <limits>
#include <queue>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

#include "gnc_core/planner.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace a_star_planner
{
const double LETHAL_OBSTACLE{static_cast<double>(nav2_costmap_2d::LETHAL_OBSTACLE)};

const int directions[8][2]{
    {1,0},
    {1,1},
    {0,1},
    {-1,1},
    {-1,0},
    {-1,-1},
    {0,-1},
    {1,-1}
};

struct coordMap
{
    int x, y;
};

struct AStarNode
{
    int x{0}, y{0};
    double g{std::numeric_limits<double>::max()};
    double h{std::numeric_limits<double>::max()};
    double f{std::numeric_limits<double>::max()};
    AStarNode* parent{nullptr};
    bool in_queue{false};
};

struct cmp
{
    bool operator()(const AStarNode* n1, const AStarNode* n2)
    {
        return n1->f > n2->f;
    }
};

class AStarGlobalPlanner : public gnc_core::Planner
{
public:
    explicit AStarGlobalPlanner();
    ~AStarGlobalPlanner();
    void configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent,
        std::shared_ptr<tf2_ros::Buffer> tf,
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
    void activate() override;
    void deactivate() override;
    void cleanup() override;
    nav_msgs::msg::Path computePath(
        const geometry_msgs::msg::PoseStamped& start,
        const geometry_msgs::msg::PoseStamped& goal) override;

private:
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
    nav2_costmap_2d::Costmap2D* costmap_;
    int size_x_;
    int size_y_;
    int prev_size_x_{0};
    int prev_size_y_{0};

    rclcpp::Logger logger_{rclcpp::get_logger("AStarGlobalPlanner")};
    rclcpp::Clock::SharedPtr clock_;

    coordMap start_m_;
    coordMap goal_m_;
    nav_msgs::msg::Path path_;

    // a* star stuff
    std::priority_queue<AStarNode*, std::vector<AStarNode*>, cmp> q_;
    std::vector<AStarNode> node_pool_;
    std::vector<AStarNode>::iterator node_pool_it_;
    std::vector<AStarNode*> node_grid_;

    void resetNodes();
    void getNeighbors(AStarNode* cur_node);
    void backtrace(AStarNode* cur_node);
    
    void addNodeToGrid(const int x, const int y, AStarNode* node)
    {
        node_grid_[size_x_ * y + x] = node;
    }

    AStarNode* getNode(const int x, const int y)
    {
        return node_grid_[size_x_ * y + x];
    }

    double getTraversalCost(const int x, const int y)
    {
        return costmap_->getCost(x, y) / LETHAL_OBSTACLE;
    }

    double getHCost(const int x, const int y)
    {
        return std::hypot(goal_m_.x - x, goal_m_.y - y);
    }
};
}

#endif