#include "tf2/utils.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "dwa_util/conversions.hpp"

namespace dwa_util
{
nav_2d_msgs::msg::Twist2D twist3Dto2D(const geometry_msgs::msg::Twist& twist)
{
    nav_2d_msgs::msg::Twist2D twist2d;
    twist2d.x = twist.linear.x;
    twist2d.y = twist.linear.y;
    twist2d.theta = twist.angular.z;
    return twist2d;
}

geometry_msgs::msg::Twist twist2Dto3D(const nav_2d_msgs::msg::Twist2D& twist)
{
    geometry_msgs::msg::Twist twist3d;
    twist3d.linear.x = twist.x;
    twist3d.linear.y = twist.y;
    twist3d.linear.z = 0.0;
    twist3d.angular.x = 0.0;
    twist3d.angular.y = 0.0;
    twist3d.angular.z = twist.theta;
    return twist3d;
}

geometry_msgs::msg::Pose2D pose3Dto2D(const geometry_msgs::msg::Pose& pose)
{
    geometry_msgs::msg::Pose2D pose2d;
    pose2d.x = pose.position.x;
    pose2d.y = pose.position.y;
    pose2d.theta = tf2::getYaw(pose.orientation);
    return pose2d;
}

nav_2d_msgs::msg::Path2D path3Dto2D(const nav_msgs::msg::Path& path)
{
    nav_2d_msgs::msg::Path2D path2d;
    path2d.header.frame_id = path.header.frame_id;
    for (size_t i=0; i<path.poses.size(); ++i)
    {
        geometry_msgs::msg::Pose2D pose2d;
        pose2d.x = path.poses[i].pose.position.x;
        pose2d.y = path.poses[i].pose.position.y;
        pose2d.theta = tf2::getYaw(path.poses[i].pose.orientation);
        path2d.poses.push_back(pose2d);
    }
    return path2d;
}

nav_msgs::msg::Path path2Dto3D(const nav_2d_msgs::msg::Path2D& path)
{
    nav_msgs::msg::Path path3d;
    path3d.header.frame_id = path.header.frame_id;
    geometry_msgs::msg::PoseStamped pose3d;
    pose3d.pose.position.z = 0.0;
    pose3d.pose.orientation.x = 0.0;
    pose3d.pose.orientation.y = 0.0;
    for (size_t i=0; i<path.poses.size(); ++i)
    {
        pose3d.pose.position.x = path.poses[i].x;
        pose3d.pose.position.y = path.poses[i].y;
        pose3d.pose.position.z = 0.0;
    
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, path.poses[i].theta);
        pose3d.pose.orientation.z = q.z();
        pose3d.pose.orientation.w = q.w();

        path3d.poses.push_back(pose3d);
    }
    return path3d;
}
}