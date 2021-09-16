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

void RobotFsm::Init(IRobotCtrl& robotCtrl, IRobotEngineObserver& robotEngineObserver) {
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

ERESULT RobotFsm::StartBuildMap(string task_id) {
    ROS_INFO("RobotFsm::startBuildMap entry");
    ERESULT res = state_->StartBuildMap(task_id);
    ROS_INFO("RobotFsm::startBuildMap exit");
    return res;
}

ERESULT RobotFsm::StopBuildMap(string task_id) {
    ROS_INFO("RobotFsm::stopBuildMap entry");
    ERESULT res = state_->StopBuildMap(task_id);
    ROS_INFO("RobotFsm::stopBuildMap exit");
    return res;
}

ERESULT RobotFsm::PauseBuildMap(string task_id) {
    ROS_INFO("RobotFsm::pauseBuildMap entry");
    ERESULT res = state_->PauseBuildMap(task_id);
    ROS_INFO("RobotFsm::pauseBuildMap exit");
    return res;
}

ERESULT RobotFsm::ResumeBuildMap(string task_id) {
    ROS_INFO("RobotFsm::resumeBuildMap entry");
    ERESULT res = state_->ResumeBuildMap(task_id);
    ROS_INFO("RobotFsm::resumeBuildMap exit");
    return res;
}

ERESULT RobotFsm::StartNavi(string task_id, ENAVITYPE type, string goal_name, PixelPose goal_pose) {
    ROS_INFO("RobotFsm::startNavi entry");
    ERESULT res = state_->StartNavi(task_id, type, goal_name, goal_pose);
    return E_OK;

}
ERESULT RobotFsm::StopNavi(string task_id) {
    return E_OK;
}

ERESULT RobotFsm::PauseNavi(string task_id) {
    return E_OK;
}

ERESULT RobotFsm::ResumeNavi(string task_id) {
    return E_OK;
}
	
ERESULT RobotFsm::onNaviDone(string task_id, int32 state, string description) {
    ROS_INFO("RobotFsm::onNaviDone entry");
    ERESULT res = state_->NaviDoneEvent(task_id, state, description);
    ROS_INFO("RobotFsm::onNaviDone exit");
    return E_OK;
}

ERESULT RobotFsm::onNaviActive(string task_id, bool active) {
    ROS_INFO("RobotFsm::onNaviActive entry");
    return E_OK;
}

ERESULT RobotFsm::onNaviProgress(string task_id, Pose curr_pose) {
    ROS_INFO("RobotFsm::onNaviProgress entry");
    return E_OK;
}

} // end_ns

