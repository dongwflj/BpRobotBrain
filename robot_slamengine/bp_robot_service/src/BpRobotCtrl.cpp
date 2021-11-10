/*
* Copyright (c) 2021, BytePowers, Inc.
* All rights reserved.
*
* author: Ewen Dong
* Created Date: 2021-08-30
*/
#include <ros/ros.h>

#include "BpRobotCtrl.h"
#include "robot_observer.h"

namespace bp {

BpRobotCtrl::BpRobotCtrl()
{
}

BpRobotCtrl::~BpRobotCtrl()
{
}

ERESULT BpRobotCtrl::Init() {
    ROS_INFO("BpRobotCtrl::Init entry");
    observer_->OnInitDone();
    ROS_INFO("BpRobotCtrl::Init exit");
    return E_OK;
}

ERESULT BpRobotCtrl::StartBuildMap(const std::string& task_id) {
    ROS_INFO("BpRobotCtrl::startBuildMap entry");
    ROS_INFO("BpRobotCtrl::startBuildMap exit");
    return E_OK;
}

ERESULT BpRobotCtrl::StopBuildMap( const std::string& task_id) {
    ROS_INFO("BpRobotCtrl::stopBuildMap entry");
    ROS_INFO("BpRobotCtrl::stopBuildMap exit");
    return E_OK;
}

ERESULT BpRobotCtrl::PauseBuildMap(const std::string& task_id) {
    ROS_INFO("BpRobotCtrl::pauseBuildMap entry");
    ROS_INFO("BpRobotCtrl::pauseBuildMap exit");
    // TODO: need to support the function
    return E_NOTSUPPORT;
}

ERESULT BpRobotCtrl::ResumeBuildMap(const std::string& task_id) {
    ROS_INFO("BpRobotCtrl::resumeBuildMap entry");
    ROS_INFO("BpRobotCtrl::resumeBuildMap exit");
    // TODO: need to support the function
    return E_NOTSUPPORT;
}

ERESULT BpRobotCtrl::SaveMap(const std::string& task_id, const std::string& save_path) {
    ROS_INFO("BpRobotCtrl::saveBuildMap entry");
    ROS_INFO("BpRobotCtrl::saveBuildMap exit");
    return E_OK;
}

ERESULT BpRobotCtrl::LoadMap(const std::string& task_id, const std::string& map_path) {
    ROS_INFO("BpRobotCtrl::loadBuildMap entry");
    ROS_INFO("BpRobotCtrl::loadBuildMap exit");
    return E_OK;
}

ERESULT BpRobotCtrl::StartNavi(const std::string& task_id, ENAVITYPE type, const std::string& goal_name, const PixelPose& goal_pose){
    ROS_INFO("BpRobotCtrl::startNavi entry");
    observer_->OnNaviDone(task_id, 0, "");
    ROS_INFO("BpRobotCtrl::startNavi exit");
    return E_OK;
}

ERESULT BpRobotCtrl::StopNavi(const std::string& task_id){
    ROS_INFO("BpRobotCtrl::StopNavi entry");
    ROS_INFO("BpRobotCtrl::StopNavi exit");
    return E_OK;
}


ERESULT BpRobotCtrl::SetObserver(IRobotObserver& observer) {
    observer_ = &observer;
    return E_OK;
}

ERESULT BpRobotCtrl::CheckPose(const std::string& poi_name, const PixelPose &poi_pose) {
    ROS_INFO("BpRobotCtrl::CheckPose entry");
    return E_OK;
}

ERESULT BpRobotCtrl::SetPixelPose(const PixelPose &pixel_pose) {
    ROS_INFO("BpRobotCtrl::SetPixelPose entry");
    return E_OK;
}

ERESULT BpRobotCtrl::SetMaxNaviVel(double max_linear_vel, double max_angular_max) {
    ROS_INFO("BpRobotCtrl::SetMaxNaviVel entry");
    return E_OK;
}

}
