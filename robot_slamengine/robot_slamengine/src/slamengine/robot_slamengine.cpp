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
#include "slamengine/robot_bt.h"

namespace slamengine
{

RobotSlamEngine::RobotSlamEngine() {
}

RobotSlamEngine::~RobotSlamEngine() {
    ROS_INFO("~RobotSlamEngine() entry");
    delete robotFsm_; 
    ROS_INFO("~RobotSlamEngine() exit");
}

void RobotSlamEngine::Init(IRobotCtrl& robotCtrl, IRobotEngineObserver& robotEngineObserver) {
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
    robotFsm_->Init(robotCtrl, robotEngineObserver);

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
 
    ROS_INFO("RobotSlamEngine::init exit");
}

ERESULT RobotSlamEngine::StartBuildMap(string task_id) {
    ROS_INFO("RobotSlamEngine::startBuildMap entry");
    return robotFsm_->StartBuildMap(task_id);
}

ERESULT RobotSlamEngine::StopBuildMap(string task_id) {
    ROS_INFO("RobotSlamEngine::stopBuildMap entry");
    return robotFsm_->StopBuildMap(task_id);
}

ERESULT RobotSlamEngine::PauseBuildMap(string task_id) {
    ROS_INFO("RobotSlamEngine::pauseBuildMap entry");
    return robotFsm_->PauseBuildMap(task_id);
}

ERESULT RobotSlamEngine::ResumeBuildMap(string task_id) {
    ROS_INFO("RobotSlamEngine::resumeBuildMap entry");
    return robotFsm_->ResumeBuildMap(task_id);
}

ERESULT RobotSlamEngine::SaveMap(string task_id, string map_name){
    ROS_INFO("RobotSlamEngine::saveMap entry");
    //return robotFsm_->SaveMap();
}

ERESULT RobotSlamEngine::LoadMap(string task_id, string map_name){
    ROS_INFO("RobotSlamEngine::loadMap entry");
    //return robotFsm_->LoadMap();
}

ERESULT RobotSlamEngine::StartNavi(string task_id, ENAVITYPE type, string name, PixelPose pose){
    ROS_INFO("RobotSlamEngine::startNavi entry");
    return robotFsm_->StartNavi(task_id, type, name, pose);
}

ERESULT RobotSlamEngine::StopNavi(string task_id){
    return E_OK;
}

ERESULT RobotSlamEngine::PauseNavi(string task_id){
    return E_OK;
}

ERESULT RobotSlamEngine::ResumeNavi(string task_id){
    return E_OK;
}
	
} // end_ns

