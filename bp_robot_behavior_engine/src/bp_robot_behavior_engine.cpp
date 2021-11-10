/*
* Copyright (c) 2021, BytePowers, Inc.
* All rights reserved.
*
* author: Ewen Dong
* Created Date: 2021-08-30
*/
#include <stdexcept>
#include <ros/ros.h>

#include "bp_robot_behavior_engine.h"
#include "bt/robot_bt.h"

namespace bp
{

BpRobotBehaviorEngine::BpRobotBehaviorEngine() {
}

BpRobotBehaviorEngine::~BpRobotBehaviorEngine() {
    ROS_INFO("~BpRobotBehaviorEngine() entry");
    //delete robotFsm_; 
    delete robotBt_; 
    ROS_INFO("~BpRobotBehaviorEngine() exit");
}

void BpRobotBehaviorEngine::Init(IRobotCtrl& robotCtrl, IRobotEngineObserver& robotEngineObserver) {
    ROS_INFO("BpRobotBehaviorEngine::Init entry");
    if (robotBt_ != nullptr) {
        ROS_ERROR("BpRobotBehaviorEngine::init re-entry");
        return;
    }
    robotBt_ = new RobotBt();
    if (robotBt_ == nullptr) {
        ROS_ERROR("BpRobotBehaviorEngine::init allocate fsm object failed");
        throw std::runtime_error("BpRobotBehaviorEngine::init Can't allocate fsm object");
    }
    robotBt_->Init(robotCtrl, robotEngineObserver);
    ROS_INFO("BpRobotBehaviorEngine::Init exit");
}

ERESULT BpRobotBehaviorEngine::StartBuildMap(const std::string& task_id) {
    ROS_INFO("BpRobotBehaviorEngine::StartBuildMap entry");
    return robotBt_->StartBuildMap(task_id);
}

ERESULT BpRobotBehaviorEngine::StopBuildMap(const std::string& task_id) {
    ROS_INFO("BpRobotBehaviorEngine::StopBuildMap entry");
    return robotBt_->StopBuildMap(task_id);
}

ERESULT BpRobotBehaviorEngine::PauseBuildMap(const std::string& task_id) {
    ROS_INFO("BpRobotBehaviorEngine::PauseBuildMap entry");
    return E_OK;
}

ERESULT BpRobotBehaviorEngine::ResumeBuildMap(const std::string& task_id) {
    ROS_INFO("BpRobotBehaviorEngine::ResumeBuildMap entry");
    return E_OK;
}

ERESULT BpRobotBehaviorEngine::SaveMap(const std::string& task_id, const std::string& map_name){
    ROS_INFO("BpRobotBehaviorEngine::SaveMap entry");
    ROS_INFO("BpRobotBehaviorEngine::SaveMap exit");
    return E_OK;
}

ERESULT BpRobotBehaviorEngine::LoadMap(const std::string& task_id, const std::string& map_name){
    ROS_INFO("BpRobotBehaviorEngine::LoadMap entry");
    return E_OK;
}

ERESULT BpRobotBehaviorEngine::StartNavi(const std::string& task_id, ENAVITYPE type, const std::string& name, const PixelPose& pose){
    ROS_INFO("BpRobotBehaviorEngine::StartNavi entry");
    return robotBt_->StartNavi(task_id, type, name, pose);
}

ERESULT BpRobotBehaviorEngine::StopNavi(const std::string& task_id){
    ROS_INFO("BpRobotBehaviorEngine::StopNavi entry");
    return robotBt_->StopNavi(task_id);
}

ERESULT BpRobotBehaviorEngine::PauseNavi(const std::string& task_id){
    ROS_INFO("BpRobotBehaviorEngine::PauseNavi entry");
    return E_OK;
}

ERESULT BpRobotBehaviorEngine::ResumeNavi(const std::string& task_id){
    ROS_INFO("BpRobotBehaviorEngine::ResumeNavi entry");
    return E_OK;
}
	

ERESULT BpRobotBehaviorEngine::NotifyDropEvent(DropType type){
    ROS_INFO("BpRobotBehaviorEngine::NotifyDropEvent entry");
    return E_OK;
}

ERESULT BpRobotBehaviorEngine::NotifyCollisionEvent(CollisionType type){
    ROS_INFO("BpRobotBehaviorEngine::NotifyCollisionEvent entry");
    return E_OK;
}

ERESULT BpRobotBehaviorEngine::NotifyEStopEvent(EmergencyType type){
    ROS_INFO("BpRobotBehaviorEngine::NotifyStopEvent entry");
    return E_OK;
}

ERESULT BpRobotBehaviorEngine::NotifyChargingEvent(const BatteryState& state){
    ROS_INFO("BpRobotBehaviorEngine::NotifyChargingEvent entry");
    return E_OK;
}

ERESULT BpRobotBehaviorEngine::NotifyCriticalHwErrEvent(){
    ROS_INFO("BpRobotBehaviorEngine::NotifyCriticalHwErrEvent entry");
    return E_OK;
}

} // end_ns
