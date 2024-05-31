#ifndef PATH_LENGTH_HPP_
#define PATH_LENGTH_HPP_

#include "geometry_msgs/msg/pose2_d.hpp"

#include "nav_2d_msgs/msg/path2_d.hpp"
#include "dwa_critics/base_critic.hpp"

namespace dwa_critics
{
/**
 * @class PathLengthCritic
 * @brief Penalizes short paths. Short paths makes the robot travel slower.
*/
class PathLengthCritic : public BaseCritic
{
public:
    PathLengthCritic();
    void prepare(const nav_2d_msgs::msg::Path2D& globa_path) override;
    double scorePath(const nav_2d_msgs::msg::Path2D& local_path) override;
    virtual ~PathLengthCritic() = default;
private:
    void on_initialize() override;
};
}

#endif