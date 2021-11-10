/*
* Copyright (c) 2021, BytePowers, Inc.
* All rights reserved.
*
* author: Ewen Dong
* Created Date: 2021-08-30
*/

#include <ros/ros.h>
#include "robot_fsm.h"
#include "robot_idle_state.h"
#include "robot_mapping_state.h"

namespace slamengine
{

ERESULT RobotState::StartBuildMap(const std::string& task_id) {
    ROS_INFO("RobotState::startBuildMap trigger at error state");
    return E_BADSTATE;
}

ERESULT RobotState::StopBuildMap(const std::string& task_id) {
    ROS_INFO("RobotState::stopBuildMap trigger at error state");
    return E_BADSTATE;
}

ERESULT RobotState::PauseBuildMap(const std::string& task_id) {
    ROS_INFO("RobotState::pauseBuildMap trigger at error state");
    return E_BADSTATE;
}

ERESULT RobotState::ResumeBuildMap(const std::string& task_id) {
    ROS_INFO("RobotState::resumeBuildMap trigger at error state");
    return E_BADSTATE;
}

ERESULT RobotState::SaveMap(const std::string& task_id, const std::string& map_name) {
    ROS_INFO("RobotState::SaveMap trigger at error state");
}

ERESULT RobotState::LoadMap(const std::string& task_id, const std::string& map_name) {
    ROS_INFO("RobotState::LoadMap trigger at error state");
}

ERESULT RobotState::StartNavi(const std::string& task_id, ENAVITYPE type, const std::string& goal_name, const PixelPose& goal_pose) {
    return E_BADSTATE;
}

ERESULT RobotState::StopNavi(const std::string& task_id) {
    return E_BADSTATE;
}

ERESULT RobotState::PauseNavi(const std::string& task_id) {
    return E_BADSTATE;
}

ERESULT RobotState::ResumeNavi(const std::string& task_id) {
    return E_BADSTATE;
}

ERESULT RobotState::InitDoneEvent() {
    ROS_INFO("RobotState::InitDoneEvent trigger at error state");
    return E_BADSTATE;
}

ERESULT RobotState::NaviDoneEvent(const std::string& task_id, int32 state, const std::string& description) {
    ROS_INFO("RobotState::NaviDoneEvent trigger at error state");
    return E_BADSTATE;
}

ERESULT RobotState::NaviActiveEvent(const std::string& task_id, bool active) {
    ROS_INFO("RobotState::naviActiveEvent trigger at error state");
    return E_BADSTATE;
}

ERESULT RobotState::NaviProgressEvent(const std::string& task_id, const Pose& curr_pose) {
    ROS_INFO("RobotState::naviProgressEvent trigger at error state");
    return E_BADSTATE;
}

ERESULT RobotState::Resume() {
    ROS_INFO("RobotState::Resume trigger, this state in no action");
    return E_BADSTATE;
}

} // end_ns

