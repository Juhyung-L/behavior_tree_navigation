#include <OgreManualObject.h>

#include "rviz_common/validate_floats.hpp"
#include "rviz_common/msg_conversions.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/properties/vector_property.hpp"
#include "rviz_rendering/material_manager.hpp"

#include "gnc_rviz_plugins/dwa_trajectories_display.hpp"

namespace gnc_rviz_plugins
{
DWATrajectoriesDisplay::DWATrajectoriesDisplay(rviz_common::DisplayContext* display_context)
: DWATrajectoriesDisplay()
{
    context_ = display_context;
    scene_manager_ = context_->getSceneManager();
    scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
}

DWATrajectoriesDisplay::DWATrajectoriesDisplay()
{
    alpha_property_ = new rviz_common::properties::FloatProperty(
        "Alpha", 1.0,
        "Amount of transparency to apply to the trajectories.", this);
    color_.a = alpha_property_->getFloat();
    color_.b = 0.0;

    offset_property_ = new rviz_common::properties::VectorProperty(
        "Offset", Ogre::Vector3::ZERO,
        "Allows you to offset the trajectories from the origin of the reference frame.  In meters.",
        this, SLOT(updateOffset()));

    static int count = 0;
    std::string material_name = "Trajectory2DMaterial" + std::to_string(count++);
    lines_material_ = rviz_rendering::MaterialManager::createMaterialWithNoLighting(material_name);
}

DWATrajectoriesDisplay::~DWATrajectoriesDisplay()
{
    destroyObjects();
}

void DWATrajectoriesDisplay::onInitialize()
{
    MFDClass::onInitialize();

    buffer_length_ = 1;
    updateBufferLength();
}

void DWATrajectoriesDisplay::reset()
{
    MFDClass::reset();
    updateBufferLength();
}

// this function is called everytime a new message arrives and needs to be displayed
void DWATrajectoriesDisplay::processMessage(nav_2d_msgs::msg::DWATrajectories::ConstSharedPtr msg)
{
    if (msg->paths.empty() || msg->scores.empty())
    {
        return;
    }

    // check if message has valid floats
    if (!validateMsg(*msg))
    {
        setStatus(rviz_common::properties::StatusProperty::Error,
            "Topic",
            "Message contained invalid floating point values (nans or infs)");
        return;
    }
    
    /**
     * RViz has a fixed frame that you can set.
     * This code block checks if there is a transform between the fixed frame and the frame of the
     * message you are trying to display.
     * If there is no transform available, function exits permaturely
    */
    Ogre::Vector3 position;
    Ogre::Quaternion orientation;
    if (!context_->getFrameManager()->getTransform(msg->header, position, orientation))
    {
        setMissingTransformToFixedFrame(msg->header.frame_id);
        return ;
    }
    setTransformOk();

    Ogre::Matrix4 transform(orientation);
    transform.setTrans(position);

    // need detach and reattach all manual objects
    // to prevent previous trajectories drawings from lingering on the scene
    scene_node_->detachAllObjects();
    for (auto& manual_object : manual_objects_)
    {
        manual_object->clear();
        scene_node_->attachObject(manual_object);
    }

    // make sure msg->paths.size() == manual_objects_.size()
    // this will be called once since the number of trajectories in the message will remain constant
    if (buffer_length_ != msg->paths.size())
    {
        buffer_length_ = msg->paths.size();
        updateBufferLength();
    }

    // get max and min total_score
    double max_score = msg->scores[0];
    double min_score = msg->scores[0];
    for (size_t i=1; i<msg->scores.size(); ++i)
    {
        if (msg->scores[i] > max_score)
        {
            max_score = msg->scores[i];
        }
        if (msg->scores[i] < min_score)
        {
            min_score = msg->scores[i]; 
        }
    }
    double diff = max_score - min_score;

    // update manual objects
    Ogre::ManualObject* manual_object = nullptr;
    nav_2d_msgs::msg::Path2D path;
    Ogre::Vector3 ogre_pose;
    ogre_pose.z = 0.0;
    for (size_t i=0 ; i<buffer_length_; ++i)
    {
        manual_object = manual_objects_[i];
        path = msg->paths[i];
        manual_object->estimateVertexCount(path.poses.size());
        manual_object->begin(
            lines_material_->getName(), Ogre::RenderOperation::OT_LINE_STRIP, "rviz_rendering");
        // set color based on score
        color_.g = (msg->scores[i] - min_score) / diff;
        color_.r = 1.0 - (msg->scores[i] - min_score) / diff;

        for (auto& pose : path.poses)
        {
            ogre_pose.x = pose.x;
            ogre_pose.y = pose.y;
            manual_object->position(transform * ogre_pose);
            rviz_rendering::MaterialManager::enableAlphaBlending(lines_material_, color_.a);
            manual_object->colour(color_);
        }
        manual_object->end();
    }

    // trigger re-render
    context_->queueRender();
}

bool DWATrajectoriesDisplay::validateMsg(const nav_2d_msgs::msg::DWATrajectories& msg)
{
    for (auto& path : msg.paths)
    {
        for (auto& pose : path.poses)
        {
            if (!rviz_common::validateFloats(pose.x) ||
                !rviz_common::validateFloats(pose.y) ||
                !rviz_common::validateFloats(pose.theta))
            {
                return false;
            }
        }
    }
    return true;
}

void DWATrajectoriesDisplay::updateOffset()
{
    scene_node_->setPosition(offset_property_->getVector());
    context_->queueRender();
}

void DWATrajectoriesDisplay::destroyObjects()
{
    for (auto manual_object : manual_objects_)
    {
        manual_object->clear();
        scene_manager_->destroyManualObject(manual_object);
    }
    manual_objects_.clear();
}

void DWATrajectoriesDisplay::updateBufferLength()
{
    destroyObjects();

    manual_objects_.reserve(buffer_length_);
    for (size_t i=0; i<buffer_length_; ++i)
    {
        auto manual_object = scene_manager_->createManualObject();
        manual_objects_.push_back(manual_object);
    }
}
}

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(gnc_rviz_plugins::DWATrajectoriesDisplay, rviz_common::Display)