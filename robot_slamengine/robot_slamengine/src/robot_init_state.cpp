/*
* Copyright (c) 2021, Cloudminds, Inc.
* All rights reserved.
*
* author: Ewen Dong
* Created Date: 2021-08-30
*/

#include <ros/ros.h>
#include "robot_init_state.h"
#include "robot_idle_state.h"
#include "robot_fsm.h"

namespace slamengine
{

ERESULT RobotInitState::InitDoneEvent() {
    ROS_INFO("RobotInitState::InitDoneEvent entry");
    context_->TransitionTo(RobotIdleState::GetInstance());
    return E_OK;
}

} // end_ns

