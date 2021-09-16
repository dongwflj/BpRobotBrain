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

ERESULT PatrolCtrl::StartBuildMap(string task_id) {
    return E_NOTSUPPORT;
}

ERESULT PatrolCtrl::StopBuildMap(string task_id) {
    ROS_INFO("PatrolCtrl::stopBuildMap entry");
    ROS_INFO("PatrolCtrl::stopBuildMap exit");
    return E_OK;
}

ERESULT PatrolCtrl::PauseBuildMap(string task_id) {
    return E_NOTSUPPORT;
}

ERESULT PatrolCtrl::ResumeBuildMap(string task_id) {
    return E_NOTSUPPORT;
}

ERESULT PatrolCtrl::SaveMap(string task_id) {
    return E_NOTSUPPORT;
}

ERESULT PatrolCtrl::LoadMap(string task_id) {
    return E_NOTSUPPORT;
}


ERESULT PatrolCtrl::StartNavi(string task_id, ENAVITYPE type, string goal_name, PixelPose goal_pose){
    ROS_INFO("PatrolCtrl::startNavi entry");
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
    ROS_INFO("PatrolCtrl::startNavi exit");
    return E_OK;
}

ERESULT PatrolCtrl::setObserver(IRobotObserver& observer) {
    observer_ = &observer;
    return E_OK;
}

}
