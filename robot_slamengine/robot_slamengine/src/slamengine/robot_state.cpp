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

ERESULT RobotState::StartBuildMap(string task_id) {
    ROS_INFO("RobotState::startBuildMap trigger at error state");
    return E_BADSTATE;
}

ERESULT RobotState::StopBuildMap(string task_id) {
    ROS_INFO("RobotState::stopBuildMap trigger at error state");
    return E_BADSTATE;
}

ERESULT RobotState::PauseBuildMap(string task_id) {
    ROS_INFO("RobotState::pauseBuildMap trigger at error state");
    return E_BADSTATE;
}

ERESULT RobotState::ResumeBuildMap(string task_id) {
    ROS_INFO("RobotState::resumeBuildMap trigger at error state");
    return E_BADSTATE;
}

ERESULT RobotState::StartNavi(string task_id, ENAVITYPE type, string goal_name, PixelPose goal_pose) {
    return E_BADSTATE;
}

ERESULT RobotState::StopNavi(string task_id) {
    return E_BADSTATE;
}

ERESULT RobotState::PauseNavi(string task_id) {
    return E_BADSTATE;
}

ERESULT RobotState::ResumeNavi(string task_id) {
    return E_BADSTATE;
}

ERESULT RobotState::NaviDoneEvent(string task_id, int32 state, string description) {
    ROS_INFO("RobotState::NaviDoneEvent trigger at error state");
    return E_BADSTATE;
}

ERESULT RobotState::NaviActiveEvent(string task_id, bool active) {
    ROS_INFO("RobotState::naviActiveEvent trigger at error state");
    return E_BADSTATE;
}

ERESULT RobotState::NaviProgressEvent(string task_id, Pose curr_pose) {
    ROS_INFO("RobotState::naviProgressEvent trigger at error state");
    return E_BADSTATE;
}
 
} // end_ns

