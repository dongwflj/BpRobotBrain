/*
* Copyright (c) 2021, Cloudminds, Inc.
* All rights reserved.
*
* author: Ewen Dong
* Created Date: 2021-08-30
*/
#include <ros/ros.h>

#include "GLiteCtrl.h"
#include "robot_observer.h"

namespace ginger {

GliteCtrl::GliteCtrl()
{
}

GliteCtrl::~GliteCtrl()
{
}

ERESULT GliteCtrl::Init() {
    ROS_INFO("GliteCtrl::Init entry");
    observer_->OnInitDone();
    ROS_INFO("GliteCtrl::Init exit");
    return E_OK;
}

ERESULT GliteCtrl::StartBuildMap(const std::string& task_id) {
    ROS_INFO("GliteCtrl::startBuildMap entry");
    return E_OK;
}

ERESULT GliteCtrl::StopBuildMap( const std::string& task_id) {
    ROS_INFO("GliteCtrl::stopBuildMap entry");
    ROS_INFO("GliteCtrl::stopBuildMap exit");
    return E_OK;
}

ERESULT GliteCtrl::PauseBuildMap(const std::string& task_id) {
    ROS_INFO("GliteCtrl::pauseBuildMap entry");
    ROS_INFO("GliteCtrl::pauseBuildMap exit");
    // TODO: need to support the function
    return E_NOTSUPPORT;
}

ERESULT GliteCtrl::ResumeBuildMap(const std::string& task_id) {
    ROS_INFO("GliteCtrl::resumeBuildMap entry");
    ROS_INFO("GliteCtrl::resumeBuildMap exit");
    // TODO: need to support the function
    return E_NOTSUPPORT;
}

ERESULT GliteCtrl::SaveMap(const std::string& task_id, const std::string& save_path) {
    ROS_INFO("GliteCtrl::saveBuildMap entry");
    ROS_INFO("GliteCtrl::saveBuildMap exit");
    return E_OK;
}

ERESULT GliteCtrl::LoadMap(const std::string& task_id, const std::string& map_path) {
    ROS_INFO("GliteCtrl::loadBuildMap entry");
    ROS_INFO("GliteCtrl::loadBuildMap exit");
    return E_OK;
}

ERESULT GliteCtrl::StartNavi(const std::string& task_id, ENAVITYPE type, const std::string& goal_name, const PixelPose& goal_pose){
    ROS_INFO("GliteCtrl::startNavi entry");
    return E_OK;
}

ERESULT GliteCtrl::SetObserver(IRobotObserver& observer) {
    observer_ = &observer;
    return E_OK;
}

ERESULT GliteCtrl::CheckPose(const std::string& poi_name, const PixelPose &poi_pose) {
    ROS_INFO("GliteCtrl::CheckPose entry");
    return E_OK;
}

ERESULT GliteCtrl::SetPixelPose(const PixelPose &pixel_pose) {
    ROS_INFO("GliteCtrl::SetPixelPose entry");
    return E_OK;
}

ERESULT GliteCtrl::SetMaxNaviVel(double max_linear_vel, double max_angular_max) {
    ROS_INFO("GliteCtrl::SetMaxNaviVel entry");
    return E_OK;
}

}
