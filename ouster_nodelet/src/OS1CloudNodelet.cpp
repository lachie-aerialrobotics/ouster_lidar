// this should really be in the implementation (.cpp file)
#include <pluginlib/class_list_macros.h>

// Include your header
#include "ouster_nodelet/OS1CloudNodelet.h"

namespace ouster_nodelet

{
    void OS1CloudNodelet::onInit()
    {
        NODELET_DEBUG("Initializing nodelet...");
    };

} // namespace ouster_nodelet

PLUGINLIB_EXPORT_CLASS(ouster_nodelet::OS1CloudNodelet, nodelet::Nodelet)
