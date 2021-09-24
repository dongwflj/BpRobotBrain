/*
* Copyright (c) 2021, Cloudminds, Inc.
* All rights reserved.
*
* author: Ewen Dong
* Created Date: 2021-08-30
*/

#include <ros/ros.h>
#include "robot_mapping_state.h"
#include "robot_idle_state.h"
#include "robot_fsm.h"
#include "robot_ctrl.h"

namespace slamengine
{

ERESULT RobotMappingState::StartBuildMap(const std::string& task_id) {
    ROS_INFO("RobotMappingState::startBuildMap entry");
    ROS_INFO("RobotMappingState::startBuildMap exit");
    return E_BUILDMAPREPEAT; 
}

ERESULT RobotMappingState::StopBuildMap(const std::string& task_id) {
    ROS_INFO("RobotMappingState::stopBuildMap entry");
    // ToDo: stop map building logic
    context_->GetRobotCtrl().StopBuildMap(task_id);
    ROS_INFO("RobotMappingState::stopBuildMap exit");
    return E_OK;
}

ERESULT RobotMappingState::PauseBuildMap(const std::string& task_id) {
    ROS_INFO("RobotMappingState::pauseBuildMap entry");
    // ToDo: stop map building logic
    ROS_INFO("RobotMappingState::pauseBuildMap exit");
    return E_OK;
}

ERESULT RobotMappingState::ResumeBuildMap(const std::string& task_id) {
    ROS_INFO("RobotMappingState::resumeBuildMap entry");
    // ToDo: stop map building logic
    ROS_INFO("RobotMappingState::resumeBuildMap exit");
    return E_OK;
}

ERESULT RobotMappingState::SaveMap(const std::string& task_id, const std::string& map_name) {
    ROS_INFO("RobotState::SaveMap entry");
    context_->GetRobotCtrl().SaveMap(task_id, map_name);
    ROS_INFO("RobotState::SaveMap exit");
}

ERESULT RobotMappingState::LoadMap(const std::string& task_id, const std::string& map_name) {
    ROS_INFO("RobotState::LoadMap entry");
    context_->GetRobotCtrl().LoadMap(task_id, map_name);
    ROS_INFO("RobotState::LoadMap exit");
}

} // end_ns

