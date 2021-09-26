/*
* Copyright (c) 2021, Cloudminds, Inc.
* All rights reserved.
*
* author: Ewen Dong
* Created Date: 2021-08-30
*/
#include <stdexcept>
#include <ros/ros.h>

#include "robot_slamengine.h"
#include "robot_fsm.h"
#include "bt/robot_bt.h"

namespace slamengine
{

RobotSlamEngine::RobotSlamEngine() {
}

RobotSlamEngine::~RobotSlamEngine() {
    ROS_INFO("~RobotSlamEngine() entry");
    //delete robotFsm_; 
    delete robotBt_; 
    ROS_INFO("~RobotSlamEngine() exit");
}

void RobotSlamEngine::Init(IRobotCtrl& robotCtrl, IRobotEngineObserver& robotEngineObserver) {
    ROS_INFO("RobotSlamEngine::Init entry");
    /*if (robotFsm_ != nullptr) {
        ROS_ERROR("RobotSlamEngine::Init re-entry");
        return;
    }
    robotFsm_ = new RobotFsm();
    if (robotFsm_ == nullptr) {
        ROS_ERROR("RobotSlamEngine::Init allocate fsm object failed");
        throw std::runtime_error("RobotSlamEngine::Init Can't allocate fsm object");
    }
    robotFsm_->Init(robotCtrl, robotEngineObserver);
    */
    if (robotBt_ != nullptr) {
        ROS_ERROR("RobotSlamEngine::init re-entry");
        return;
    }
    robotBt_ = new RobotBt();
    if (robotBt_ == nullptr) {
        ROS_ERROR("RobotSlamEngine::init allocate fsm object failed");
        throw std::runtime_error("RobotSlamEngine::init Can't allocate fsm object");
    }
    robotBt_->Init(robotCtrl, robotEngineObserver);
    ROS_INFO("RobotSlamEngine::Init exit");
}

ERESULT RobotSlamEngine::StartBuildMap(const std::string& task_id) {
    ROS_INFO("RobotSlamEngine::StartBuildMap entry");
    return robotBt_->StartBuildMap(task_id);
}

ERESULT RobotSlamEngine::StopBuildMap(const std::string& task_id) {
    ROS_INFO("RobotSlamEngine::StopBuildMap entry");
    return robotBt_->StopBuildMap(task_id);
}

ERESULT RobotSlamEngine::PauseBuildMap(const std::string& task_id) {
    ROS_INFO("RobotSlamEngine::PauseBuildMap entry");
    return E_OK;
    return robotFsm_->PauseBuildMap(task_id);
}

ERESULT RobotSlamEngine::ResumeBuildMap(const std::string& task_id) {
    ROS_INFO("RobotSlamEngine::ResumeBuildMap entry");
    return E_OK;
    return robotFsm_->ResumeBuildMap(task_id);
}

ERESULT RobotSlamEngine::SaveMap(const std::string& task_id, const std::string& map_name){
    ROS_INFO("RobotSlamEngine::SaveMap entry");
    ROS_INFO("RobotSlamEngine::SaveMap exit");
    return E_OK;
    return robotFsm_->SaveMap(task_id, map_name);
}

ERESULT RobotSlamEngine::LoadMap(const std::string& task_id, const std::string& map_name){
    ROS_INFO("RobotSlamEngine::LoadMap entry");
    return E_OK;
    return robotFsm_->LoadMap(task_id, map_name);
}

ERESULT RobotSlamEngine::StartNavi(const std::string& task_id, ENAVITYPE type, const std::string& name, const PixelPose& pose){
    ROS_INFO("RobotSlamEngine::StartNavi entry");
    return robotBt_->StartNavi(task_id, type, name, pose);
}

ERESULT RobotSlamEngine::StopNavi(const std::string& task_id){
    ROS_INFO("RobotSlamEngine::StopNavi entry");
    return robotBt_->StopNavi(task_id);
}

ERESULT RobotSlamEngine::PauseNavi(const std::string& task_id){
    ROS_INFO("RobotSlamEngine::PauseNavi entry");
    return E_OK;
    return robotFsm_->PauseNavi(task_id);
}

ERESULT RobotSlamEngine::ResumeNavi(const std::string& task_id){
    ROS_INFO("RobotSlamEngine::ResumeNavi entry");
    return robotFsm_->ResumeNavi(task_id);
}
	

ERESULT RobotSlamEngine::NotifyDropEvent(DropType type){
    return E_OK;
    ROS_INFO("RobotSlamEngine::NotifyDropEvent entry");
    robotFsm_->NotifyDropEvent(type);
    return E_OK;
}

ERESULT RobotSlamEngine::NotifyCollisionEvent(CollisionType type){
    return E_OK;
    ROS_INFO("RobotSlamEngine::NotifyCollisionEvent entry");
    robotFsm_->NotifyCollisionEvent(type);
    return E_OK;
}

ERESULT RobotSlamEngine::NotifyEStopEvent(EmergencyType type){
    return E_OK;
    ROS_INFO("RobotSlamEngine::NotifyStopEvent entry");
    robotFsm_->NotifyEStopEvent(type);
    return E_OK;
}

ERESULT RobotSlamEngine::NotifyChargingEvent(const BatteryState& state){
    return E_OK;
    ROS_INFO("RobotSlamEngine::NotifyChargingEvent entry");
    return E_OK;
}

ERESULT RobotSlamEngine::NotifyCriticalHwErrEvent(){
    ROS_INFO("RobotSlamEngine::NotifyCriticalHwErrEvent entry");
    return E_OK;
    robotFsm_->NotifyCriticalHwErrEvent();
    return E_OK;
}

} // end_ns
