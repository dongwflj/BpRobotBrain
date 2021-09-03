/*
* Copyright (c) 2021, Cloudminds, Inc.
* All rights reserved.
*
* author: Ewen Dong
* Created Date: 2021-08-30
*/

#include <ros/ros.h>

#include "slamengine/robot_slamengine.h"
#include "slamengine/robot_idle_state.h"
#include "slamengine/robot_ctrl.h"

namespace slamengine
{

RobotSlamEngine::RobotSlamEngine() {
    ROS_INFO("RobotSlamEngine entry");
    ROS_INFO("RobotSlamEngine exit");
}

RobotSlamEngine::~RobotSlamEngine() {
    ROS_INFO("RobotSlamEngine entry");
    delete state_; 
    ROS_INFO("RobotSlamEngine exit");
}

void RobotSlamEngine::init(IRobotCtrl& robotCtrl) {
    robotCtrl_ = &robotCtrl;
    transitionTo(new RobotIdleState());
}

void RobotSlamEngine::transitionTo(RobotState *state) {
    ROS_INFO("Fsm Transition to %s", typeid(*state).name());
    if (state_ != nullptr) {
        delete state_;
        state_ = nullptr;
    }
    state_ = state;
    state_->set_context(this);
    ROS_INFO("Fsm Transition exit");
}

IRobotCtrl& RobotSlamEngine::getRobotCtrl() {
    return *robotCtrl_;
}

ERESULT RobotSlamEngine::startBuildMap() {
    ROS_INFO("RobotSlamEngine::startBuildMap entry");
    ERESULT res = state_->startBuildMap();
    ROS_INFO("RobotSlamEngine::startBuildMap exit");
    return res;
}

ERESULT RobotSlamEngine::stopBuildMap() {
    ROS_INFO("RobotSlamEngine::stopBuildMap entry");
    ERESULT res = state_->stopBuildMap();
    ROS_INFO("RobotSlamEngine::stopBuildMap exit");
    return res;
}

ERESULT RobotSlamEngine::pauseBuildMap() {
    ROS_INFO("RobotSlamEngine::pauseBuildMap entry");
    ERESULT res = state_->pauseBuildMap();
    ROS_INFO("RobotSlamEngine::pauseBuildMap exit");
    return res;
}

ERESULT RobotSlamEngine::resumeBuildMap() {
    ROS_INFO("RobotSlamEngine::resumeBuildMap entry");
    ERESULT res = state_->resumeBuildMap();
    ROS_INFO("RobotSlamEngine::resumeBuildMap exit");
    return res;
}

ERESULT RobotSlamEngine::onNaviDone() {
    ROS_INFO("RobotSlamEngine::onNaviDone entry");
    return E_OK;
}

ERESULT RobotSlamEngine::onNaviActive() {
    ROS_INFO("RobotSlamEngine::onNaviActive entry");
    return E_OK;
}

ERESULT RobotSlamEngine::onNaviProgress() {
    ROS_INFO("RobotSlamEngine::onNaviProgress entry");
    return E_OK;
}

} // end_ns

