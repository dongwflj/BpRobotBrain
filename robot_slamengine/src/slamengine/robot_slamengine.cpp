/*
* Copyright (c) 2021, Cloudminds, Inc.
* All rights reserved.
*
* author: Ewen Dong
* Created Date: 2021-08-30
*/
#include <stdexcept>
#include <ros/ros.h>

#include "slamengine/robot_slamengine.h"
#include "slamengine/robot_fsm.h"

namespace slamengine
{

RobotSlamEngine::RobotSlamEngine() {
}

RobotSlamEngine::~RobotSlamEngine() {
    ROS_INFO("~RobotSlamEngine() entry");
    delete robotFsm_; 
    ROS_INFO("~RobotSlamEngine() exit");
}

void RobotSlamEngine::init(IRobotCtrl& robotCtrl, IRobotEngineObserver& robotEngineObserver) {
    ROS_INFO("RobotSlamEngine::init entry");
    if (robotFsm_ != nullptr) {
        ROS_ERROR("RobotSlamEngine::init re-entry");
        return;
    }
    robotFsm_ = new RobotFsm();
    if (robotFsm_ == nullptr) {
        ROS_ERROR("RobotSlamEngine::init allocate fsm object failed");
        throw std::runtime_error("RobotSlamEngine::init Can't allocate fsm object");
    }
    robotFsm_->init(robotCtrl, robotEngineObserver);
    ROS_INFO("RobotSlamEngine::init exit");
}

ERESULT RobotSlamEngine::startBuildMap() {
    ROS_INFO("RobotSlamEngine::startBuildMap entry");
    return robotFsm_->startBuildMap();
}

ERESULT RobotSlamEngine::stopBuildMap() {
    ROS_INFO("RobotSlamEngine::stopBuildMap entry");
    return robotFsm_->stopBuildMap();
}

ERESULT RobotSlamEngine::pauseBuildMap() {
    ROS_INFO("RobotSlamEngine::pauseBuildMap entry");
    return robotFsm_->pauseBuildMap();
}

ERESULT RobotSlamEngine::resumeBuildMap() {
    ROS_INFO("RobotSlamEngine::resumeBuildMap entry");
    return robotFsm_->resumeBuildMap();
}

ERESULT RobotSlamEngine::startNavi(ENAVITYPE type) {
    ROS_INFO("RobotSlamEngine::startNavi entry");
    return robotFsm_->startNavi(type);
}

ERESULT RobotSlamEngine::stopNavi() {
    return E_OK;
}

ERESULT RobotSlamEngine::pauseNavi() {
    return E_OK;
}

ERESULT RobotSlamEngine::resumeNavi() {
    return E_OK;
}
	
} // end_ns

