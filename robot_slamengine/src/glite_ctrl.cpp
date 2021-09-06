/*
* Copyright (c) 2021, Cloudminds, Inc.
* All rights reserved.
*
* author: Ewen Dong
* Created Date: 2021-08-30
*/
#include <ros/ros.h>

#include "glite_ctrl.h"
#include "slamengine/robot_observer.h"

namespace ginger {

ERESULT GliteCtrl::startBuildMap() {
    return E_NOTSUPPORT;
}

ERESULT GliteCtrl::stopBuildMap() {
    ROS_INFO("GliteCtrl::stopBuildMap entry");
    ROS_INFO("GliteCtrl::stopBuildMap exit");
    return E_OK;
}

ERESULT GliteCtrl::pauseBuildMap() {
    return E_NOTSUPPORT;
}

ERESULT GliteCtrl::resumeBuildMap() {
    return E_NOTSUPPORT;
}

ERESULT GliteCtrl::saveMap() {
    return E_NOTSUPPORT;
}

ERESULT GliteCtrl::loadMap() {
    return E_NOTSUPPORT;
}

ERESULT GliteCtrl::startNavi(ENAVITYPE type) {
    ROS_INFO("GliteCtrl::startNavi entry");
    switch(type) {
    case NAVI_NORMAL:
        //TODO:
        observer_->onNaviDone();
        break;
    case NAVI_GOCHARGE:
        //TODO:Charging behavior
        observer_->onNaviDone();
        break;
    case NAVI_UNCHARGE:
        //TODO:Run undocking behavior then navi
        observer_->onNaviDone();
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
