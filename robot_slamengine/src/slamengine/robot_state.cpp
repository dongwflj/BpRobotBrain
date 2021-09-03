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

ERESULT RobotState::pauseBuildMap() {
    ROS_INFO("RobotState::pauseBuildMap trigger at error state");
    return E_BADSTATE;
}

ERESULT RobotState::resumeBuildMap() {
    ROS_INFO("RobotState::resumeBuildMap trigger at error state");
    return E_BADSTATE;
}

ERESULT RobotState::StartNavi(ENAVITYPE type) {
    return E_BADSTATE;
}

ERESULT RobotState::StopNavi() {
    return E_BADSTATE;
}

ERESULT RobotState::PauseNavi() {
    return E_BADSTATE;
}

ERESULT RobotState::ResumeNavi() {
    return E_BADSTATE;
}
	
ERESULT RobotState::onNaviDone() {
    ROS_INFO("RobotState::onNaviDone trigger at error state");
	return E_BADSTATE; 
}

ERESULT RobotState::onNaviActive() {
    ROS_INFO("RobotState::onNaviActive trigger at error state");
	return E_BADSTATE; 
}

ERESULT RobotState::onNaviProgress() {
    ROS_INFO("RobotState::onNaviProgress trigger at error state");
	return E_BADSTATE; 
}

} // end_ns

