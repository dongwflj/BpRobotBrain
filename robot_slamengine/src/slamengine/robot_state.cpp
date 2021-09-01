/*
* Copyright (c) 2021, Cloudminds, Inc.
* All rights reserved.
*
* author: Ewen Dong
* Created Date: 2021-08-30
*/

#include <ros/ros.h>
#include "slamengine/robot_idle_state.h"
#include "slamengine/robot_mapping_state.h"

namespace slamengine
{

ERESULT RobotState::startBuildMap() {
    ROS_INFO("RobotState::startBuildMap trigger at error state");
    return E_BADSTATE;
}

ERESULT RobotState::stopBuildMap() {
    ROS_INFO("RobotState::stopBuildMap trigger at error state");
    return E_BADSTATE;
}

} // end_ns

