/*
* Copyright (c) 2021, BytePowers, Inc.
* All rights reserved.
*
* author: Ewen Dong
* Created Date: 2021-08-30
*/
#include <stdexcept>
#include <ros/ros.h>

#include "robot_fsm.h"
#include "robot_init_state.h"
#include "robot_idle_state.h"
#include "robot_ctrl.h"
#include "robot_engine_observer.h"

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
    robotCtrl_->SetObserver(*this);
    TransitionTo(RobotInitState::GetInstance());    
    robotCtrl_->Init();
    ROS_INFO("RobotFsm init exit");
}

void RobotFsm::TransitionTo(RobotState &state) {
    ROS_INFO("Fsm Transition to %s", typeid(state).name());
    state_ = &state;
    state_->SetContext(this);
    ROS_INFO("Fsm Transition exit");
}

IRobotCtrl& RobotFsm::GetRobotCtrl() {
	if(robotCtrl_ == nullptr) {
        ROS_ERROR("robotCtrl pointer is invalid");
		throw std::invalid_argument("Invalid 'robotCtrl' pointer."); 
	}
    return *robotCtrl_;
}

IRobotEngineObserver& RobotFsm::GetRobotEngineObserver() {
    if(robotEngineObserver_ == nullptr) {
        throw std::invalid_argument("Invalid 'robotEngineObserver' pointer."); 
    }
    return *robotEngineObserver_;
}

ERESULT RobotFsm::StartBuildMap(const std::string& task_id) {
    ROS_INFO("RobotFsm::startBuildMap entry");
    ERESULT res = state_->StartBuildMap(task_id);
    ROS_INFO("RobotFsm::startBuildMap exit");
    return res;
}

ERESULT RobotFsm::StopBuildMap(const std::string& task_id) {
    ROS_INFO("RobotFsm::stopBuildMap entry");
    ERESULT res = state_->StopBuildMap(task_id);
    ROS_INFO("RobotFsm::stopBuildMap exit");
    return res;
}

ERESULT RobotFsm::PauseBuildMap(const std::string& task_id) {
    ROS_INFO("RobotFsm::pauseBuildMap entry");
    ERESULT res = state_->PauseBuildMap(task_id);
    ROS_INFO("RobotFsm::pauseBuildMap exit");
    return res;
}

ERESULT RobotFsm::ResumeBuildMap(const std::string& task_id) {
    ROS_INFO("RobotFsm::resumeBuildMap entry");
    ERESULT res = state_->ResumeBuildMap(task_id);
    ROS_INFO("RobotFsm::resumeBuildMap exit");
    return res;
}

ERESULT RobotFsm::SaveMap(const std::string& task_id, const std::string& map_name) {
    ROS_INFO("RobotFsm::SaveMap entry");
    ERESULT res = state_->SaveMap(task_id, map_name);
    ROS_INFO("RobotFsm::SaveMap exit");
    return res;
}

ERESULT RobotFsm::LoadMap(const std::string& task_id, const std::string& map_name) {
    ROS_INFO("RobotFsm::LoadMap entry");
    ERESULT res = state_->LoadMap(task_id, map_name);
    ROS_INFO("RobotFsm::LoadMap exit");
    return res;
}

ERESULT RobotFsm::StartNavi(const std::string& task_id, ENAVITYPE type, const std::string& goal_name, const PixelPose& goal_pose) {
    ROS_INFO("RobotFsm::startNavi entry");
    ERESULT res = state_->StartNavi(task_id, type, goal_name, goal_pose);
    ROS_INFO("RobotFsm::startNavi exit");
    return E_OK;

}
ERESULT RobotFsm::StopNavi(const std::string& task_id) {
    ROS_INFO("RobotFsm::stopNavi entry");
    ERESULT res = state_->StopNavi(task_id);
    ROS_INFO("RobotFsm::stopNavi exit");
    return E_OK;
}

ERESULT RobotFsm::PauseNavi(const std::string& task_id) {
    ROS_INFO("RobotFsm::pauseNavi entry");
    ERESULT res = state_->PauseNavi(task_id);
    ROS_INFO("RobotFsm::pauseNavi exit");
    return E_OK;
}

ERESULT RobotFsm::ResumeNavi(const std::string& task_id) {
    ROS_INFO("RobotFsm::resumeNavi entry");
    ERESULT res = state_->ResumeNavi(task_id);
    ROS_INFO("RobotFsm::resumeNavi exit");
    return E_OK;
}
	

ERESULT RobotFsm::Resume() {
    ROS_INFO("RobotFsm::Resume entry");
    ERESULT res = state_->Resume();
    ROS_INFO("RobotFsm::Resume exit");
    return res;
}

ERESULT RobotFsm::OnInitDone() {
    ROS_INFO("RobotFsm::onInitDone entry");
    ERESULT res = state_->InitDoneEvent();
    ROS_INFO("RobotFsm::onInitDone exit");
    return res;
}

ERESULT RobotFsm::OnNaviDone(const std::string& task_id, int32 state, const std::string& description) {
    ROS_INFO("RobotFsm::onNaviDone entry");
    ERESULT res = state_->NaviDoneEvent(task_id, state, description);
    ROS_INFO("RobotFsm::onNaviDone exit");
    return res;
}

ERESULT RobotFsm::OnNaviActive(const std::string& task_id, bool active) {
    ROS_INFO("RobotFsm::onNaviActive entry");
    ERESULT res = state_->NaviActiveEvent(task_id, active);
    ROS_INFO("RobotFsm::onNaviActive exit");
    return res;
}

ERESULT RobotFsm::OnNaviProgress(const std::string& task_id, const Pose& curr_pose) {
    ROS_INFO("RobotFsm::onNaviProgress entry");
    ERESULT res = state_->NaviProgressEvent(task_id, curr_pose);
    ROS_INFO("RobotFsm::onNaviProgress exit");
    return res;
}

ERESULT RobotFsm::NotifyDropEvent(DropType type){
    ROS_INFO("RobotFsm::NotifyDropEvent entry");
    ERESULT res = state_->NotifyDropEvent(type);
    ROS_INFO("RobotFsm::NotifyDropEvent exit");
    return res;
}

ERESULT RobotFsm::NotifyCollisionEvent(CollisionType type){
    ROS_INFO("RobotFsm::NotifyCollisionEvent entry");
    ERESULT res = state_->NotifyCollisionEvent(type);
    ROS_INFO("RobotFsm::NotifyCollisionEvent exit");
    return res;
}

ERESULT RobotFsm::NotifyEStopEvent(EmergencyType type){
    ROS_INFO("RobotFsm::NotifyEStopEvent entry");
    ERESULT res = state_->NotifyEStopEvent(type);
    ROS_INFO("RobotFsm::NotifyEStopEvent exit");
    return res;
}

ERESULT RobotFsm::NotifyChargingEvent(const BatteryState& state){
    ROS_INFO("RobotFsm::NotifyChargingEvent entry");
    ERESULT res = state_->NotifyChargingEvent(state);
    ROS_INFO("RobotFsm::NotifyChargingEvent exit");
    return res;
}

ERESULT RobotFsm::NotifyCriticalHwErrEvent(){
    ROS_INFO("RobotFsm::NotifyCriticalHwErrEvent entry");
    ROS_INFO("RobotFsm::NotifyCriticalHwErrEvent exit");
    return E_OK;
}

} // end_ns

