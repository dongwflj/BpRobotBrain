/*
* Copyright (c) 2021, Cloudminds, Inc.
* All rights reserved.
*
* author: Ewen Dong
* Created Date: 2021-08-30
*/

#include <ros/ros.h>
#include "slamengine/robot_engine_observer.h"
#include "slamengine/robot_motioning_state.h"
#include "slamengine/robot_mapping_state.h"
#include "slamengine/robot_idle_state.h"
#include "slamengine/robot_fsm.h"
#include "slamengine/robot_ctrl.h"

namespace slamengine
{

ERESULT RobotMotioningState::naviDoneEvent() {
    ROS_INFO("RobotMotioningState::naviDoneEvent entry");
    context_->getRobotEngineObserver().onNaviDone();
    context_->transitionTo(RobotIdleState::getInstance());    
    ROS_INFO("RobotMotioningState::naviDoneEvent exit");
	return E_OK; 
}

} // end_ns

