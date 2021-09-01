/*
* Copyright (c) 2021, Cloudminds, Inc.
* All rights reserved.
*
* author: Ewen Dong
* Created Date: 2021-08-30
*/

#include <ros/ros.h>
#include "slamengine/robot_mapping_state.h"
#include "slamengine/robot_idle_state.h"

namespace slamengine
{

ERESULT RobotMappingState::startBuildMap() {
    ROS_INFO("RobotMappingState::startBuildMap entry");
    ROS_INFO("RobotMappingState::startBuildMap exit");
    return E_BUILDMAPREPEAT; 
}

ERESULT RobotMappingState::stopBuildMap() {
    ROS_INFO("RobotMappingState::stopBuildMap entry");
    // ToDo: stop map building logic
    context_->TransitionTo(new RobotIdleState());    
    ROS_INFO("RobotMappingState::stopBuildMap exit");
    return E_OK;
}

} // end_ns

