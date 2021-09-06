/*
* Copyright (c) 2021, Cloudminds, Inc.
* All rights reserved.
*
* author: Ewen Dong
* Created Date: 2021-08-30
*/
#include <stdexcept>
#include <ros/ros.h>

#include "slamengine/robot_fsm.h"
#include "slamengine/robot_idle_state.h"
#include "slamengine/robot_ctrl.h"
#include "slamengine/robot_engine_observer.h"

namespace slamengine
{

RobotFsm::~RobotFsm() {
    ROS_INFO("RobotFsm entry");
    ROS_INFO("RobotFsm exit");
}

void RobotFsm::init(IRobotCtrl& robotCtrl, IRobotEngineObserver& robotEngineObserver) {
    ROS_INFO("RobotFsm init entry");
    robotCtrl_ = &robotCtrl;
    robotEngineObserver_ = &robotEngineObserver;
    robotCtrl_->setObserver(*this);
    transitionTo(RobotIdleState::getInstance());    
    ROS_INFO("RobotFsm init exit");
}

void RobotFsm::transitionTo(RobotState &state) {
    ROS_INFO("Fsm Transition to %s", typeid(state).name());
    state_ = &state;
    state_->set_context(this);
    ROS_INFO("Fsm Transition exit");
}

IRobotCtrl& RobotFsm::getRobotCtrl() {
	if(robotCtrl_ == nullptr) {
        ROS_ERROR("robotCtrl pointer is invalid");
		throw std::invalid_argument("Invalid 'robotCtrl' pointer."); 
	}
    return *robotCtrl_;
}

IRobotEngineObserver& RobotFsm::getRobotEngineObserver() {
	if(robotEngineObserver_ == nullptr) {
		throw std::invalid_argument("Invalid 'robotEngineObserver' pointer."); 
	}
    return *robotEngineObserver_;
}

ERESULT RobotFsm::startBuildMap() {
    ROS_INFO("RobotFsm::startBuildMap entry");
    ERESULT res = state_->startBuildMap();
    ROS_INFO("RobotFsm::startBuildMap exit");
    return res;
}

ERESULT RobotFsm::stopBuildMap() {
    ROS_INFO("RobotFsm::stopBuildMap entry");
    ERESULT res = state_->stopBuildMap();
    ROS_INFO("RobotFsm::stopBuildMap exit");
    return res;
}

ERESULT RobotFsm::pauseBuildMap() {
    ROS_INFO("RobotFsm::pauseBuildMap entry");
    ERESULT res = state_->pauseBuildMap();
    ROS_INFO("RobotFsm::pauseBuildMap exit");
    return res;
}

ERESULT RobotFsm::resumeBuildMap() {
    ROS_INFO("RobotFsm::resumeBuildMap entry");
    ERESULT res = state_->resumeBuildMap();
    ROS_INFO("RobotFsm::resumeBuildMap exit");
    return res;
}

ERESULT RobotFsm::startNavi(ENAVITYPE type) {
    ROS_INFO("RobotFsm::startNavi entry");
    ERESULT res = state_->startNavi(type);
    return E_OK;

}
ERESULT RobotFsm::stopNavi() {
    return E_OK;
}

ERESULT RobotFsm::pauseNavi() {
    return E_OK;
}

ERESULT RobotFsm::resumeNavi() {
    return E_OK;
}
	
ERESULT RobotFsm::onNaviDone() {
    ROS_INFO("RobotFsm::onNaviDone entry");
    ERESULT res = state_->naviDoneEvent();
    ROS_INFO("RobotFsm::onNaviDone exit");
    return E_OK;
}

ERESULT RobotFsm::onNaviActive() {
    ROS_INFO("RobotFsm::onNaviActive entry");
    return E_OK;
}

ERESULT RobotFsm::onNaviProgress() {
    ROS_INFO("RobotFsm::onNaviProgress entry");
    return E_OK;
}

} // end_ns

