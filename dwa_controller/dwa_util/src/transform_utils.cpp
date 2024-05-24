#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Vector3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/exceptions.h"

#include "dwa_util/transform_utils.hpp"

namespace dwa_util
{
bool transformPath2D(
    const std::shared_ptr<tf2_ros::Buffer> tf,
    const std::string& target_frame,
    const nav_2d_msgs::msg::Path2D& in_path,
    nav_2d_msgs::msg::Path2D& out_path)
{
    if (in_path.header.frame_id == target_frame)
    {
        out_path = in_path;
        return true;
    }

    geometry_msgs::msg::TransformStamped transform;
    try
    {
        transform = tf->lookupTransform(target_frame, in_path.header.frame_id, tf2::TimePointZero);
    }
    catch (tf2::TransformException& ex)
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("tf_help"),
            "Exception in transformPose: %s",
            ex.what());
        return false;
    }

    tf2::Quaternion q(
        transform.transform.rotation.x,
        transform.transform.rotation.y,
        transform.transform.rotation.z,
        transform.transform.rotation.w
    );
    tf2::Matrix3x3 rot(q);
    tf2::Vector3 t(
        transform.transform.translation.x,
        transform.transform.translation.y,
        transform.transform.translation.z
    );
    tf2::Vector3 in_pose;
    tf2::Vector3 out_pose;

    out_path.poses.resize(in_path.poses.size());
    for (size_t i=0; i<in_path.poses.size(); ++i)
    {
        in_pose.setX(in_path.poses[i].x);
        in_pose.setY(in_path.poses[i].y);
        in_pose.setZ(0.0);

        out_pose = rot * in_pose + t;

        out_path.poses[i].x = out_pose.x();
        out_path.poses[i].y = out_pose.y();
    }
    return true;
}
}