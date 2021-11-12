/*
* Copyright (c) 2021, BytePowers, Inc.
* All rights reserved.
*
* author: Ewen Dong
* Created Date: 2021-08-30
*/
#include <stdexcept>
#include <ros/ros.h>

#include "robot_behavior_engine.h"
#include "bt/robot_bt.h"

namespace bp
{

RobotBehaviorEngine::RobotBehaviorEngine() {
}

RobotBehaviorEngine::~RobotBehaviorEngine() {
    ROS_INFO("~RobotBehaviorEngine() entry");
    //delete robotFsm_; 
    delete robotBt_; 
    ROS_INFO("~RobotBehaviorEngine() exit");
}

ERESULT RobotBehaviorEngine::Init(IRobotCtrl& robotCtrl, IRobotEngineObserver& robotEngineObserver) {
    ROS_INFO("RobotBehaviorEngine::Init entry");
    if (robotBt_ != nullptr) {
        ROS_ERROR("RobotBehaviorEngine::init re-entry");
        return E_FALSE;
    }
    robotBt_ = new RobotBt();
    if (robotBt_ == nullptr) {
        ROS_ERROR("RobotBehaviorEngine::init allocate fsm object failed");
        throw std::runtime_error("RobotBehaviorEngine::init Can't allocate fsm object");
    }
    robotBt_->Init(robotCtrl, robotEngineObserver);
    ROS_INFO("RobotBehaviorEngine::Init exit");
    return E_OK;
}

ERESULT RobotBehaviorEngine::StartBuildMap(const std::string& task_id) {
    ROS_INFO("RobotBehaviorEngine::StartBuildMap entry");
    return robotBt_->StartBuildMap(task_id);
}

ERESULT RobotBehaviorEngine::StopBuildMap(const std::string& task_id) {
    ROS_INFO("RobotBehaviorEngine::StopBuildMap entry");
    return robotBt_->StopBuildMap(task_id);
}

ERESULT RobotBehaviorEngine::PauseBuildMap(const std::string& task_id) {
    ROS_INFO("RobotBehaviorEngine::PauseBuildMap entry");
    return E_OK;
}

ERESULT RobotBehaviorEngine::ResumeBuildMap(const std::string& task_id) {
    ROS_INFO("RobotBehaviorEngine::ResumeBuildMap entry");
    return E_OK;
}

ERESULT RobotBehaviorEngine::SaveMap(const std::string& task_id, const std::string& map_name){
    ROS_INFO("RobotBehaviorEngine::SaveMap entry");
    ROS_INFO("RobotBehaviorEngine::SaveMap exit");
    return E_OK;
}

ERESULT RobotBehaviorEngine::LoadMap(const std::string& task_id, const std::string& map_name){
    ROS_INFO("RobotBehaviorEngine::LoadMap entry");
    return E_OK;
}

ERESULT RobotBehaviorEngine::StartNavi(const std::string& task_id, ENAVITYPE type, const std::string& name, const PixelPose& pose){
    ROS_INFO("RobotBehaviorEngine::StartNavi entry");
    return robotBt_->StartNavi(task_id, type, name, pose);
}

ERESULT RobotBehaviorEngine::StopNavi(const std::string& task_id){
    ROS_INFO("RobotBehaviorEngine::StopNavi entry");
    return robotBt_->StopNavi(task_id);
}

ERESULT RobotBehaviorEngine::PauseNavi(const std::string& task_id){
    ROS_INFO("RobotBehaviorEngine::PauseNavi entry");
    return E_OK;
}

ERESULT RobotBehaviorEngine::ResumeNavi(const std::string& task_id){
    ROS_INFO("RobotBehaviorEngine::ResumeNavi entry");
    return E_OK;
}
	

ERESULT RobotBehaviorEngine::NotifyDropEvent(DropType type){
    ROS_INFO("RobotBehaviorEngine::NotifyDropEvent entry");
    return E_OK;
}

ERESULT RobotBehaviorEngine::NotifyCollisionEvent(CollisionType type){
    ROS_INFO("RobotBehaviorEngine::NotifyCollisionEvent entry");
    return E_OK;
}

ERESULT RobotBehaviorEngine::NotifyEStopEvent(EmergencyType type){
    ROS_INFO("RobotBehaviorEngine::NotifyStopEvent entry");
    return E_OK;
}

ERESULT RobotBehaviorEngine::NotifyChargingEvent(const BatteryState& state){
    ROS_INFO("RobotBehaviorEngine::NotifyChargingEvent entry");
    return E_OK;
}

ERESULT RobotBehaviorEngine::NotifyCriticalHwErrEvent(){
    ROS_INFO("RobotBehaviorEngine::NotifyCriticalHwErrEvent entry");
    return E_OK;
}

} // end_ns
