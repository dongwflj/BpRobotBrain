/*
* Copyright (c) 2021, Cloudminds, Inc.
* All rights reserved.
*
* author: Ewen Dong
* Created Date: 2021-08-30
*/

#include <ros/ros.h>

#include "slamengine/robot_slamengine.h"
#include "slamengine/robot_fsm.h"
#include "slamengine/robot_idle_state.h"

namespace slamengine
{

RobotSlamEngine::RobotSlamEngine() {
    ROS_INFO("RobotSlamEngine entry");
    fsm_ = new RobotFsm(new RobotIdleState());
    ROS_INFO("RobotSlamEngine exit");
}

ERESULT RobotSlamEngine::startBuildMap() {
    ROS_INFO("RobotSlamEngine::StartBuildMap entry");
    ROS_INFO("RobotSlamEngine::StartBuildMap exit");
    return fsm_->startBuildMap();
}

ERESULT RobotSlamEngine::stopBuildMap() {
    ROS_INFO("RobotSlamEngine::StopBuildMap entry");
    ROS_INFO("RobotSlamEngine::StopBuildMap exit");
    return fsm_->stopBuildMap();
}


} // end_ns

