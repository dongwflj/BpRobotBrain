/*
* Copyright (c) 2021, Cloudminds, Inc.
* All rights reserved.
*
* author: Ewen Dong
* Created Date: 2021-08-30
*/
#include <ros/ros.h>

#include "PatrolCtrl.h"
#include "slamengine/robot_observer.h"

namespace ginger {

ERESULT PatrolCtrl::Init() {
    ROS_INFO("PatrolCtrl::Init entry");
    observer_->OnInitDone();
    ROS_INFO("PatrolCtrl::Init exit");
    return E_OK;
}

ERESULT PatrolCtrl::StartBuildMap(const std::string& task_id) {
    return E_NOTSUPPORT;
}

ERESULT PatrolCtrl::StopBuildMap(const std::string& task_id) {
    ROS_INFO("PatrolCtrl::stopBuildMap entry");
    ROS_INFO("PatrolCtrl::stopBuildMap exit");
    return E_OK;
}

ERESULT PatrolCtrl::PauseBuildMap(const std::string& task_id) {
    return E_NOTSUPPORT;
}

ERESULT PatrolCtrl::ResumeBuildMap(const std::string& task_id) {
    return E_NOTSUPPORT;
}

ERESULT PatrolCtrl::SaveMap(const std::string& task_id, const std::string& map_name) {
    return E_NOTSUPPORT;
}

ERESULT PatrolCtrl::LoadMap(const std::string& task_id, const std::string& map_name) {
    return E_NOTSUPPORT;
}


ERESULT PatrolCtrl::StartNavi(const std::string& task_id, ENAVITYPE type, const std::string& goal_name, const PixelPose& goal_pose){
    ROS_INFO("PatrolCtrl::startNavi entry");
    switch(type) {
    case NAVI_NORMAL:
        //TODO:
        observer_->OnNaviDone(task_id, 0, "");
        break;
    case NAVI_GOCHARGE:
        //TODO:Charging behavior
        observer_->OnNaviDone(task_id, 0, "");
        break;
    case NAVI_UNCHARGE:
        //TODO:Run undocking behavior then navi
        observer_->OnNaviDone(task_id, 0, "");
        break;
    default:
        break;
    }
    ROS_INFO("PatrolCtrl::startNavi exit");
    return E_OK;
}

ERESULT PatrolCtrl::SetObserver(IRobotObserver& observer) {
    observer_ = &observer;
    return E_OK;
}

}
