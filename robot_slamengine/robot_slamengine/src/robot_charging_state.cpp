/*
* Copyright (c) 2021, Cloudminds, Inc.
* All rights reserved.
*
* author: Ewen Dong
* Created Date: 2021-08-30
*/

#include <ros/ros.h>
#include "robot_charging_state.h"
#include "robot_mapping_state.h"
#include "robot_fsm.h"
#include "robot_ctrl.h"

namespace slamengine
{

ERESULT RobotChargingState::StartBuildMap(const std::string& task_id) {
    ROS_INFO("RobotChargingState::startBuildMap entry");
    // Impl build map operation
//    context_->TransitionTo(new RobotMappingState());    
    ROS_INFO("RobotChargingState::startBuildMap exit");
    return E_OK;
}

} // end_ns

