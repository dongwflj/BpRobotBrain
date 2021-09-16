/*
* Copyright (c) 2021, Cloudminds, Inc.
* All rights reserved.
*
* author: Ewen Dong
* Created Date: 2021-08-30
*/
#include <ros/ros.h>

#include "GLiteCtrl.h"
#include "slamengine/robot_observer.h"

namespace ginger {

ERESULT GliteCtrl::StartBuildMap(string task_id) {
    return E_NOTSUPPORT;
}

ERESULT GliteCtrl::StopBuildMap(string task_id) {
    ROS_INFO("GliteCtrl::stopBuildMap entry");
    ROS_INFO("GliteCtrl::stopBuildMap exit");
    return E_OK;
}

ERESULT GliteCtrl::PauseBuildMap(string task_id) {
    return E_NOTSUPPORT;
}

ERESULT GliteCtrl::ResumeBuildMap(string task_id) {
    return E_NOTSUPPORT;
}

ERESULT GliteCtrl::SaveMap(string task_id) {
    return E_NOTSUPPORT;
}

ERESULT GliteCtrl::LoadMap(string task_id) {
    return E_NOTSUPPORT;
}

ERESULT GliteCtrl::StartNavi(string task_id, ENAVITYPE type, string goal_name, PixelPose goal_pose){
    ROS_INFO("GliteCtrl::startNavi entry");
    switch(type) {
    case NAVI_NORMAL:
        //TODO:
        observer_->onNaviDone(task_id, 0, "");
        break;
    case NAVI_GOCHARGE:
        //TODO:Charging behavior
        observer_->onNaviDone(task_id, 0, "");
        break;
    case NAVI_UNCHARGE:
        //TODO:Run undocking behavior then navi
        observer_->onNaviDone(task_id, 0, "");
        break;
    default:
        break;
    }
    ROS_INFO("GliteCtrl::startNavi exit");
    return E_OK;
}

ERESULT GliteCtrl::setObserver(IRobotObserver& observer) {
    observer_ = &observer;
    return E_OK;
}

}
