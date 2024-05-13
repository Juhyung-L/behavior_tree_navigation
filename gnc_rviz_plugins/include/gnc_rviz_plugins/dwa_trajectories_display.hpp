#ifndef GNC_RVIZ_PLUGINS__DWA_TRAJECTORIES_DISPLAY_HPP_
#define GNC_RVIZ_PLUGINS__DWA_TRAJECTORIES_DISPLAY_HPP_

#include <vector>
#include <memory>

#include "rviz_common/message_filter_display.hpp"
#include "rviz_rendering/objects/line.hpp"
#include "rviz_common/message_filter_display.hpp"

#include "nav_2d_msgs/msg/dwa_trajectories.hpp"

namespace Ogre
{
class ManualObject;
}

namespace rviz_common
{
namespace properties
{
class FloatProperty;
class VectorProperty;
}
}

namespace gnc_rviz_plugins
{
class DWATrajectoriesDisplay : public rviz_common::MessageFilterDisplay<nav_2d_msgs::msg::DWATrajectories>
{
    Q_OBJECT

public:
    DWATrajectoriesDisplay(rviz_common::DisplayContext* display_context);
    DWATrajectoriesDisplay();
    ~DWATrajectoriesDisplay() override;

    void processMessage(nav_2d_msgs::msg::DWATrajectories::ConstSharedPtr msg) override;
protected:
    void onInitialize() override;
    void reset() override;

private Q_SLOTS:
    void updateOffset();

private:
    void updateBufferLength();
    bool validateMsg(const nav_2d_msgs::msg::DWATrajectories& msg);
    void destroyObjects();

    std::vector<Ogre::ManualObject*> manual_objects_;
    Ogre::MaterialPtr lines_material_;

    rviz_common::properties::FloatProperty* alpha_property_;
    rviz_common::properties::VectorProperty* offset_property_;

    Ogre::ColourValue color_;
    size_t buffer_length_;
};
}

#endif 