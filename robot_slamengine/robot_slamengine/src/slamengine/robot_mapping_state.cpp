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
#include "slamengine/robot_fsm.h"
#include "slamengine/robot_ctrl.h"

namespace slamengine
{

ERESULT RobotMappingState::StartBuildMap(string task_id) {
    ROS_INFO("RobotMappingState::startBuildMap entry");
    ROS_INFO("RobotMappingState::startBuildMap exit");
    return E_BUILDMAPREPEAT; 
}

ERESULT RobotMappingState::StopBuildMap(string task_id) {
    ROS_INFO("RobotMappingState::stopBuildMap entry");
    // ToDo: stop map building logic
    context_->getRobotCtrl().StopBuildMap(task_id);
    context_->transitionTo(RobotIdleState::getInstance());    
    ROS_INFO("RobotMappingState::stopBuildMap exit");
    return E_OK;
}

ERESULT RobotMappingState::PauseBuildMap(string task_id) {
    ROS_INFO("RobotMappingState::pauseBuildMap entry");
    // ToDo: stop map building logic
    ROS_INFO("RobotMappingState::pauseBuildMap exit");
    return E_OK;
}

ERESULT RobotMappingState::ResumeBuildMap(string task_id) {
    ROS_INFO("RobotMappingState::resumeBuildMap entry");
    // ToDo: stop map building logic
    ROS_INFO("RobotMappingState::resumeBuildMap exit");
    return E_OK;
}


} // end_ns

