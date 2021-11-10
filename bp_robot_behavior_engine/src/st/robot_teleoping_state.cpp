/*
* Copyright (c) 2021, BytePowers, Inc.
* All rights reserved.
*
* author: Ewen Dong
* Created Date: 2021-08-30
*/

#include <ros/ros.h>
#include "robot_teleoping_state.h"
#include "robot_mapping_state.h"
#include "robot_fsm.h"
#include "robot_ctrl.h"

namespace slamengine
{

ERESULT RobotTeleOpingState::StartBuildMap(const std::string& task_id) {
    ROS_INFO("RobotTeleOpingState::startBuildMap entry");
    // Impl build map operation
    context_->TransitionTo(RobotMappingState::GetInstance());    
    ROS_INFO("RobotTeleOpingState::startBuildMap exit");
    return E_OK;
}

} // end_ns

