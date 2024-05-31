#ifndef GLOBAL_PATH_ALIGN_HPP_
#define GLOBAL_PATH_ALIGN_HPP_

#include <vector>
#include <queue>

#include "geometry_msgs/msg/pose2_d.hpp"

#include "nav_2d_msgs/msg/path2_d.hpp"
#include "dwa_critics/base_critic.hpp"

namespace dwa_critics
{
struct Node
{
    Node(unsigned int idx)
    : idx(idx)
    {}

    Node(unsigned int idx, unsigned char parent_val)
    : idx(idx)
    , parent_val(parent_val)
    {}

    unsigned int idx;
    unsigned char parent_val;
};
/**
 * @class GlobalPathAlignCritic
 * @brief A critic that penalizes paths that do not align with the global path
*/
class GlobalPathAlignCritic : public BaseCritic 
{
public:
    GlobalPathAlignCritic();
    void prepare(const nav_2d_msgs::msg::Path2D& global_path) override;
    double scorePath(const nav_2d_msgs::msg::Path2D& local_path) override;
    virtual ~GlobalPathAlignCritic() = default;

private:
    void on_initialize() override;
    void makeAlignmentMap(const nav_2d_msgs::msg::Path2D& path);
    std::vector<unsigned char> alignment_map_;
    unsigned int size_x_;
    unsigned int size_y_;
};
}

#endif