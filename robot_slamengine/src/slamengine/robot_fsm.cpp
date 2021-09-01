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
#include "slamengine/robot_idle_state.h"

namespace slamengine
{

RobotFsm::RobotFsm(RobotState *state) : state_(nullptr) {
    if (state == nullptr) {
        ROS_ERROR("RobotFsm init as Invalid 'event' argument.");
        ///throw std::invalid_argument("RobotFsm init as Invalid 'event' argument.");
        return;
    }
    this->TransitionTo(state);
}

RobotFsm::~RobotFsm() {
    delete state_; 
};

void RobotFsm::TransitionTo(RobotState *state) {
    ROS_INFO("Fsm Transition to %s", typeid(*state).name());
    if (state_ != nullptr) {
        delete state_;
        state_ = nullptr;
    }
    state_ = state;
    state_->set_context(this);
    ROS_INFO("Fsm Transition exit");
}

ERESULT RobotFsm::startBuildMap() {
    return state_->startBuildMap();
}

ERESULT RobotFsm::stopBuildMap() {
    return state_->stopBuildMap();
}

} // end_ns

