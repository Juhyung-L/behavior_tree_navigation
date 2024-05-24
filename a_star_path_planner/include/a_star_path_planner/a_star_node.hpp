#ifndef A_STAR_NODE_HPP_
#define A_STAR_NODE_HPP_

#include <memory>
#include <chrono>
#include <limits>
#include <queue>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace a_star
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

/**
 * @class AStar
 * @brief A ROS node that finds the path to a goal pose from the robot's current pose
*/

class AStar : public nav2_util::LifecycleNode
{
public:
    explicit AStar(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~AStar();

protected:
    nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
    nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
    nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
    nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
    nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
    std::unique_ptr<nav2_util::NodeThread> costmap_thread_;
    nav2_costmap_2d::Costmap2D * costmap_;
    int size_x_;
    int size_y_;
    int prev_size_x_{0};
    int prev_size_y_{0};

    rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Logger logger_;

    coordMap goal_;
    coordMap src_;
    nav_msgs::msg::Path path_;
    bool goal_received_;

    // a* star stuff
    std::priority_queue<AStarNode*, std::vector<AStarNode*>, cmp> q_;
    std::vector<AStarNode> node_pool_;
    std::vector<AStarNode>::iterator node_pool_it_;
    std::vector<AStarNode*> node_grid_;

    void goalCB(const geometry_msgs::msg::PoseStamped goal);
    void computePath();
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
        return std::hypot(x - goal_.x, y - goal_.y);
    }
};
}

#endif