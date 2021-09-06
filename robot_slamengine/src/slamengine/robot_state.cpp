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

ERESULT RobotState::startNavi(ENAVITYPE type) {
    return E_BADSTATE;
}

ERESULT RobotState::stopNavi() {
    return E_BADSTATE;
}

ERESULT RobotState::pauseNavi() {
    return E_BADSTATE;
}

ERESULT RobotState::resumeNavi() {
    return E_BADSTATE;
}

ERESULT RobotState::naviDoneEvent() {
    ROS_INFO("RobotState::naviDoneEvent trigger at error state");
    return E_BADSTATE;
}

ERESULT RobotState::naviActiveEvent() {
    ROS_INFO("RobotState::naviActiveEvent trigger at error state");
    return E_BADSTATE;
}

ERESULT RobotState::naviProgressEvent() {
    ROS_INFO("RobotState::naviProgressEvent trigger at error state");
    return E_BADSTATE;
}
 
} // end_ns

