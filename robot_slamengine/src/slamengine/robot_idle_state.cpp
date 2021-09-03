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
#include "slamengine/robot_fsm.h"
#include "slamengine/robot_ctrl.h"

namespace slamengine
{

ERESULT RobotIdleState::startBuildMap() {
    ROS_INFO("RobotIdleState::startBuildMap entry");
    // Impl build map operation
    context_->transitionTo(new RobotMappingState());    
    ROS_INFO("RobotIdleState::startBuildMap exit");
    return E_OK;
}

} // end_ns

